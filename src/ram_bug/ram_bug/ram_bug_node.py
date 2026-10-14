"""
ram_bug_node - RAM-Bug evader for TurtleBot3 (ROS 2 Jazzy)
==========================================================
Subscribes  /scan (LaserScan), /odom (Odometry), /goal_pose (PoseStamped, optional)
Publishes   /cmd_vel (Twist or TwistStamped, auto-detected)
            /ram_bug/mode (String), /ram_bug/map (OccupancyGrid, frame 'arena'),
            /ram_bug/path (Path), /ram_bug/chaser (PointStamped), /ram_bug/target (PointStamped)

All positions are in the ARENA frame: origin at a corner of the 3 x 3 m arena.
Tell the node where the robot starts with start_x, start_y, start_yaw.
"""
import json
import math
import os

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from . import eg_core as eg
from .brain import Brain


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RamBugNode(Node):
    PARAMS = dict(
        goal_x=float("nan"), goal_y=float("nan"),          # arena metres (nan = wait for /goal_pose)
        start_x=0.15, start_y=0.15, start_yaw=0.0,          # where the robot starts in the arena
        capture_r=0.30, time_limit=75.0,
        max_lin=0.20, max_ang=2.5, k_ang=3.0, lookahead_cells=3, stop_dist=0.13, stuck_time=2.0,
        inflate_cells=1, robot_max_cells=4, robot_max_extent=3, motion_min_free=2,
        laser_x_offset=-0.032, max_free_range=3.0,
        plan_period=0.5, control_period=0.05,
        cmd_vel_stamped="auto",                            # auto / true / false
        params_file="",                                    # tuned RAM-Bug parameters (json)
        autostart=True,
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

        self.odom = None
        self.scan = None
        self.active = False
        self.cmd_pub = None
        self.stamped = None

        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "goal_pose", self.on_goal, 10)
        self.mode_pub = self.create_publisher(String, "ram_bug/mode", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "ram_bug/map", 1)
        self.path_pub = self.create_publisher(Path, "ram_bug/path", 1)
        self.chaser_pub = self.create_publisher(PointStamped, "ram_bug/chaser", 1)
        self.target_pub = self.create_publisher(PointStamped, "ram_bug/target", 1)
        self.create_timer(cfg["plan_period"], self.on_plan)
        self.create_timer(cfg["control_period"], self.on_control)

        if not math.isnan(cfg["goal_x"]) and not math.isnan(cfg["goal_y"]):
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
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_scan(self, msg):
        self.scan = (list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)

    def on_goal(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        frame = msg.header.frame_id.lstrip("/")
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
        self.brain.plan(x, y, yaw, self.scan, self.now())
        if self.brain.done and not prev_done:
            res = {"reached": "WIN - reached the goal", "timeout": "DRAW - time is up"}[self.brain.done]
            self.get_logger().info(res)
        self.mode_pub.publish(String(data=self.brain.mode))
        self.publish_debug(x, y)

    def on_control(self):
        if not self.active or self.odom is None or self.scan is None:
            return
        x, y, yaw = self.pose()
        ranges, amin, ainc, rmin, rmax = self.scan
        front = 9.9
        for i, r in enumerate(ranges):
            a = math.atan2(math.sin(amin + i * ainc), math.cos(amin + i * ainc))
            if abs(a) <= math.radians(30) and rmin < r < rmax:
                front = min(front, r)
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
    rclpy.init()
    node = RamBugNode()
    try:
        rclpy.spin(node)
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
