"""
ram_bug_node - goal-reaching evader for TurtleBot3 (ROS 2 Jazzy)
================================================================
Planner: Bug2 + LiDAR moving-object avoidance (algorithm:=bug2, default) or RAM-Bug (algorithm:=ram_bug)
Subscribes  /scan (LaserScan), /odom (Odometry), /goal_pose (PoseStamped, optional)
Publishes   /cmd_vel (Twist or TwistStamped, auto-detected)
            /ram_bug/mode (String), /ram_bug/map (OccupancyGrid, frame 'arena'),
            /ram_bug/path (Path), /ram_bug/chaser (PointStamped), /ram_bug/target (PointStamped)

All positions are in the ARENA frame: origin at a corner of the 3 x 3 m arena.
Tell the node where the robot starts with start_x, start_y, start_yaw, or set
auto_start:=true: the robot stands still for ~1 s and finds the arena walls itself
(auto_start.py), then publishes the arena -> odom transform. Then click 2D Goal Pose.
"""
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from . import eg_core as eg
from .auto_start import fit_arena, scan_points
from .brain import Brain


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RamBugNode(Node):
    PARAMS = dict(
        goal_x=float("nan"), goal_y=float("nan"),          # arena metres (nan = wait for /goal_pose)
        start_x=0.15, start_y=0.15, start_yaw=0.0,          # where the robot starts in the arena
        capture_r=0.30, time_limit=75.0,
        max_lin=0.20, max_ang=2.5, k_ang=3.0, lookahead_cells=3, stop_dist=0.17, stuck_time=2.0,
        blocked_time=0.5,                                  # s held by stop_dist -> back up and turn
        min_aim=0.25, smooth_deg=15.0,                     # m: never aim closer; deg: smooth smaller turns
        inflate_cells=1, robot_max_cells=4, robot_max_extent=3, motion_min_free=2,
        laser_x_offset=-0.032, max_free_range=3.0,
        plan_period=0.5, control_period=0.05,
        cmd_vel_stamped="auto",                            # auto / true / false
        params_file="",                                    # tuned RAM-Bug parameters (json)
        use_peer=True,                                     # use the chaser pose from peer_link_node
        peer_timeout=0.6,                                  # s; older than this -> fall back to LiDAR
        plan_margin=0.0,                                   # m added to capture_r when planning
        detect_chaser="lidar",                             # lidar / off (off = test without a chaser)
        algorithm="bug2",                                  # bug2 / ram_bug
        avoid_r=1.2, avoid_exit=1.4,                       # m; Bug2 steps away from a moving object this close
        turn_gate=0.6,                                     # rad/s; scans taken while turning faster are ignored
        forget_ticks=12,                                   # forget a LiDAR chaser not seen for 12 plans (6 s)
        motion_confirm=True,                               # a moving blob must be seen in 2 scans in a row
        autostart=True,
        auto_start=False,                                  # find start_x/y/yaw from the arena walls
        auto_scans=5,                                      # still scans used for that
        arena_size=3.0,
    )

    def __init__(self):
        super().__init__("ram_bug")
        for k, v in self.PARAMS.items():
            self.declare_parameter(k, v)
        cfg = {k: self.get_parameter(k).value for k in self.PARAMS}
        self.cfg = cfg

        tuned = None
        pf = cfg["params_file"]
        if not pf:
            try:
                from ament_index_python.packages import get_package_share_directory
                pf = os.path.join(get_package_share_directory("ram_bug"), "config", "best_params.json")
            except Exception:
                pf = ""
        if pf and os.path.exists(pf):
            with open(pf, encoding="utf-8") as f:
                tuned = json.load(f)["params"]
            self.get_logger().info(f"loaded tuned parameters from {pf}")
        self.brain = Brain(cfg, tuned)
        self.get_logger().info(f"planner: {self.brain.algorithm}")
        if eg._np is not None:
            self.get_logger().info("planner: numpy + scipy fast path")
        else:
            self.get_logger().warn("planner: pure Python (sudo apt install python3-scipy for ~2x faster planning)")
        self.localized = not cfg["auto_start"]
        self.auto_buf = []
        self.pending_goal = None
        self.tf_static = None
        if cfg["auto_start"]:
            self.get_logger().info("auto_start: keep the robot STILL - looking for the arena walls ...")

        self.odom = None
        self.scan = None
        self.active = False
        self.cmd_pub = None
        self.stamped = None

        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "goal_pose", self.on_goal, 10)
        if cfg["use_peer"]:
            self.create_subscription(PoseStamped, "ram_bug/peer_pose", self.on_peer, 10)
        self.mode_pub = self.create_publisher(String, "ram_bug/mode", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "ram_bug/map", 1)
        self.path_pub = self.create_publisher(Path, "ram_bug/path", 1)
        self.chaser_pub = self.create_publisher(PointStamped, "ram_bug/chaser", 1)
        self.target_pub = self.create_publisher(PointStamped, "ram_bug/target", 1)
        # separate groups so a slow plan (pure Python, ~0.1-0.3 s on a Pi) cannot hold up /cmd_vel
        self.create_timer(cfg["plan_period"], self.on_plan, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(cfg["control_period"], self.on_control, callback_group=MutuallyExclusiveCallbackGroup())

        if not self.localized:
            if not math.isnan(cfg["goal_x"]) and not math.isnan(cfg["goal_y"]):
                self.pending_goal = (cfg["goal_x"], cfg["goal_y"], "arena")
        elif not math.isnan(cfg["goal_x"]) and not math.isnan(cfg["goal_y"]):
            self.start_goal(cfg["goal_x"], cfg["goal_y"])
        else:
            self.get_logger().info("waiting for a goal: set goal_x/goal_y or publish /goal_pose (RViz 2D Goal Pose)")

    # ------------------------------------------------------------ cmd_vel type
    def ensure_cmd_pub(self):
        if self.cmd_pub is not None:
            return True
        mode = str(self.cfg["cmd_vel_stamped"]).lower()
        if mode in ("true", "false"):
            self.stamped = mode == "true"
        else:
            subs = self.get_subscriptions_info_by_topic("/cmd_vel")
            if not subs:
                return False                                # nobody listening yet, try again later
            self.stamped = any("TwistStamped" in s.topic_type for s in subs)
        self.cmd_pub = self.create_publisher(TwistStamped if self.stamped else Twist, "cmd_vel", 10)
        self.get_logger().info(f"/cmd_vel type: {'TwistStamped' if self.stamped else 'Twist'}")
        return True

    def send(self, v, w):
        if not self.ensure_cmd_pub():
            return
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        else:
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
        self.cmd_pub.publish(m)

    # ------------------------------------------------------------ inputs
    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = yaw_of(p.orientation)
        t = self.now()
        prev = getattr(self, "_odom_prev", None)            # turn rate from successive odometry
        if prev is not None and t - prev[1] > 1e-3:
            w = math.atan2(math.sin(yaw - prev[0]), math.cos(yaw - prev[0])) / (t - prev[1])
            self.odom_w = 0.5 * getattr(self, "odom_w", 0.0) + 0.5 * w
        self._odom_prev = (yaw, t)
        self.odom = (p.position.x, p.position.y, yaw)

    def on_scan(self, msg):
        ranges = list(msg.ranges)
        key = (len(ranges), msg.angle_min, msg.angle_increment)
        if getattr(self, "_front_key", None) != key:       # beams within +-30 deg of straight ahead
            self._front_key = key
            self.front_idx = [i for i in range(len(ranges))
                              if abs(math.atan2(math.sin(msg.angle_min + i * msg.angle_increment),
                                                math.cos(msg.angle_min + i * msg.angle_increment)))
                              <= math.radians(30)]
        self.scan = (ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        if not self.localized:
            self.auto_localize()
            return
        # remember where we were when this scan arrived (the plan runs up to 0.5 s later)
        self.scan_pose = None if self.odom is None else (*self.pose(), getattr(self, "odom_w", 0.0))

    def on_peer(self, msg):
        self.brain.set_peer(msg.pose.position.x, msg.pose.position.y, self.now())

    # ------------------------------------------------------------ auto start
    def auto_localize(self):
        if self.odom is None:
            return
        if abs(getattr(self, "odom_w", 0.0)) > 0.05:
            self.auto_buf = []                              # robot is turning: start again
            return
        self.auto_buf.append(scan_points(self.odom, *self.scan, laser_x=self.cfg["laser_x_offset"]))
        if len(self.auto_buf) < self.cfg["auto_scans"]:
            return
        fit = fit_arena(self.auto_buf, size=self.cfg["arena_size"])
        self.auto_buf = []
        if fit is None:
            self.get_logger().warn("auto_start: could not find the arena walls, trying again",
                                   throttle_duration_sec=2.0)
            return
        for k in ("start_x", "start_y", "start_yaw"):
            self.cfg[k] = fit[k]                            # Brain shares this dict
        self.publish_arena_tf()
        self.localized = True
        x, y, yaw = self.pose()
        self.get_logger().info(
            f"auto_start: robot is at ({x:.2f}, {y:.2f}) m, heading {math.degrees(yaw):.0f} deg in the arena "
            f"(walls found {fit['walls'][1] - fit['walls'][0]:.2f} x {fit['walls'][3] - fit['walls'][2]:.2f} m)")
        if self.pending_goal is not None:
            gx, gy, frame = self.pending_goal
            self.pending_goal = None
            if frame == "odom":
                gx, gy, _ = self.brain.odom_to_arena(gx, gy, 0.0)
            self.start_goal(gx, gy)
        else:
            self.get_logger().info("auto_start: now click 2D Goal Pose in RViz (Fixed Frame: arena)")

    def publish_arena_tf(self):
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        if self.tf_static is None:
            self.tf_static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "arena", "odom"
        t.transform.translation.x = float(self.cfg["start_x"])
        t.transform.translation.y = float(self.cfg["start_y"])
        yaw = float(self.cfg["start_yaw"])
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_static.sendTransform(t)

    def on_goal(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        frame = msg.header.frame_id.lstrip("/")
        if not self.localized:
            self.pending_goal = (x, y, "odom" if frame == "odom" else "arena")
            self.get_logger().info("goal saved - still looking for the arena walls (keep the robot still)")
            return
        if frame == "odom":                                 # convert odom -> arena
            pose = self.brain.odom_to_arena(x, y, 0.0)
            x, y = pose[0], pose[1]
        elif frame not in ("arena", ""):
            self.get_logger().warn(f"goal frame '{frame}' unknown, assuming arena coordinates")
        self.start_goal(x, y)

    def start_goal(self, x, y):
        try:
            self.brain.set_goal(x, y)
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        self.active = bool(self.cfg["autostart"]) or self.active
        self.get_logger().info(f"GOAL set to ({x:.2f}, {y:.2f}) m in the arena frame")

    def pose(self):
        if self.odom is None:
            return None
        return self.brain.odom_to_arena(*self.odom)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ loops
    def on_plan(self):
        if not self.active or self.scan is None or self.odom is None:
            return
        x, y, yaw = self.pose()
        prev_done = self.brain.done
        t0 = time.perf_counter()
        self.brain.plan(x, y, yaw, self.scan, self.now(), getattr(self, "scan_pose", None))
        dt = time.perf_counter() - t0
        if dt > 0.5 * self.cfg["plan_period"]:              # the robot drives on an old target meanwhile
            self.get_logger().warn(f"slow plan: {1000 * dt:.0f} ms in {self.brain.mode}", throttle_duration_sec=1.0)
        if self.brain.done and not prev_done:
            res = {"reached": "WIN - reached the goal", "timeout": "DRAW - time is up"}[self.brain.done]
            self.get_logger().info(res)
        src = "shared" if self.brain.peer_used else "lidar"
        mode = f"{self.brain.mode} (chaser from {src})"
        self.mode_pub.publish(String(data=mode))
        if mode != getattr(self, "_last_mode", None):       # print every state change to the console
            self._last_mode = mode
            t = 0.0 if self.brain.t_start is None else self.now() - self.brain.t_start
            pl = self.brain.planner
            ch = ""
            if pl is not None and pl.ch_est is not None:
                cx, cy = eg.center(pl.ch_est)
                ch = f", chaser ({cx * eg.CELL:.2f}, {cy * eg.CELL:.2f}) {math.hypot(cx * eg.CELL - x, cy * eg.CELL - y):.2f} m away"
            self.get_logger().info(f"[{t:5.1f} s] {mode} at ({x:.2f}, {y:.2f}){ch}")
        self.publish_debug(x, y)

    def on_control(self):
        if not self.active or self.odom is None or self.scan is None:
            return
        if not self.ensure_cmd_pub():
            return                                          # nobody listens yet: do not run the stuck timer
        x, y, yaw = self.pose()
        ranges, _, _, rmin, rmax = self.scan
        front = min((ranges[i] for i in self.front_idx if rmin < ranges[i] < rmax), default=9.9)
        v, w = self.brain.control(x, y, yaw, front, self.now())
        self.send(v, w)

    # ------------------------------------------------------------ RViz debug
    def publish_debug(self, x, y):
        pl = self.brain.planner
        if pl is None:
            return
        stamp = self.get_clock().now().to_msg()
        g = OccupancyGrid()
        g.header.stamp, g.header.frame_id = stamp, "arena"
        g.info.resolution = eg.CELL
        g.info.width, g.info.height = eg.W, eg.H
        data = [0] * eg.N
        for c in pl.known:
            data[c] = 60
        for c in getattr(pl, "raw", ()):
            data[c] = 100
        g.data = data
        self.map_pub.publish(g)
        path = Path()
        path.header.stamp, path.header.frame_id = stamp, "arena"
        for c in (pl.race_path or []):
            ps = PoseStamped()
            ps.header = path.header
            cx, cy = eg.center(c)
            ps.pose.position.x, ps.pose.position.y = cx * eg.CELL, cy * eg.CELL
            path.poses.append(ps)
        self.path_pub.publish(path)
        if pl.ch_est is not None:
            pt = PointStamped()
            pt.header.stamp, pt.header.frame_id = stamp, "arena"
            cx, cy = eg.center(pl.ch_est)
            pt.point.x, pt.point.y = cx * eg.CELL, cy * eg.CELL
            self.chaser_pub.publish(pt)
        if self.brain.target is not None:
            pt = PointStamped()
            pt.header.stamp, pt.header.frame_id = stamp, "arena"
            pt.point.x, pt.point.y = self.brain.target
            self.target_pub.publish(pt)


def main():
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = RamBugNode()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor(num_threads=2))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
