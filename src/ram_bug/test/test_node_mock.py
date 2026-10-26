"""
Smoke test of ram_bug_node.py WITHOUT ROS installed: replaces rclpy and the message
packages with tiny fakes, then feeds odometry + LaserScan messages and checks that
/cmd_vel commands come out. (On a machine with ROS 2 you can ignore this file.)

  python3 test/test_node_mock.py
"""
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))


# ---------------------------------------------------------------- fake ROS
class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def msgtype(name, **fields):
    def init(self, **kw):
        for k, v in fields.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)
    return type(name, (), {"__init__": init})


Vec = msgtype("Vector3", x=0.0, y=0.0, z=0.0)
Quat = msgtype("Quaternion", x=0.0, y=0.0, z=0.0, w=1.0)
Point = msgtype("Point", x=0.0, y=0.0, z=0.0)
Header = msgtype("Header", stamp=None, frame_id="")
Pose = msgtype("Pose", position=Point, orientation=Quat)
Twist = msgtype("Twist", linear=Vec, angular=Vec)
TwistStamped = msgtype("TwistStamped", header=Header, twist=Twist)
PoseStamped = msgtype("PoseStamped", header=Header, pose=Pose)
PointStamped = msgtype("PointStamped", header=Header, point=Point)
PoseWC = msgtype("PoseWithCovariance", pose=Pose)
Odometry = msgtype("Odometry", header=Header, pose=PoseWC)
LaserScan = msgtype("LaserScan", ranges=list, angle_min=0.0, angle_increment=0.0, range_min=0.12, range_max=3.5)
MapInfo = msgtype("MapMetaData", resolution=0.0, width=0, height=0)
OccupancyGrid = msgtype("OccupancyGrid", header=Header, info=MapInfo, data=list)
Path = msgtype("Path", header=Header, poses=list)
String = msgtype("String", data="")


class Pub:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []

    def publish(self, m):
        self.msgs.append(m)


class Clock:
    t = 0.0

    def now(self):
        c = self

        class T:
            nanoseconds = int(c.t * 1e9)

            def to_msg(self):
                return None
        return T()


class Param:
    def __init__(self, v):
        self.value = v


class Node:
    def __init__(self, name):
        self._params, self.pubs, self.subs, self.timers = {}, {}, {}, []
        self._clock = Clock()

    def declare_parameter(self, k, v):
        self._params[k] = v

    def get_parameter(self, k):
        return Param(self._params[k])

    def create_subscription(self, t, topic, cb, q):
        self.subs[topic] = cb

    def create_publisher(self, t, topic, q):
        self.pubs[topic] = Pub(topic)
        return self.pubs[topic]

    def create_timer(self, p, cb, callback_group=None):
        self.timers.append((p, cb))

    def get_clock(self):
        return self._clock

    def get_logger(self):
        class L:
            def info(self, m):
                print("[info]", m)
            warn = error = info
        return L()

    def get_subscriptions_info_by_topic(self, t):
        return [Msg(topic_type="geometry_msgs/msg/TwistStamped")]


def install_fakes():
    mods = {
        "rclpy": types.ModuleType("rclpy"), "rclpy.node": types.ModuleType("rclpy.node"),
        "rclpy.qos": types.ModuleType("rclpy.qos"), "rclpy.callback_groups": types.ModuleType("rclpy.callback_groups"), "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"), "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": types.ModuleType("nav_msgs.msg"), "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"), "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
    }
    mods["rclpy.node"].Node = Node
    mods["rclpy.qos"].qos_profile_sensor_data = 10
    mods["rclpy.callback_groups"].MutuallyExclusiveCallbackGroup = object
    g = mods["geometry_msgs.msg"]
    g.PointStamped, g.PoseStamped, g.Twist, g.TwistStamped = PointStamped, PoseStamped, Twist, TwistStamped
    g.Pose, g.PoseArray = Pose, msgtype("PoseArray", header=Header, poses=list)
    n = mods["nav_msgs.msg"]
    n.OccupancyGrid, n.Odometry, n.Path = OccupancyGrid, Odometry, Path
    mods["sensor_msgs.msg"].LaserScan = LaserScan
    mods["std_msgs.msg"].String = String
    g.TransformStamped = msgtype("TransformStamped", header=Header, child_frame_id=str,
                                  transform=msgtype("Transform", translation=Vec, rotation=Quat))
    tf = types.ModuleType("tf2_ros")

    class STB:
        sent = []

        def __init__(self, node):
            pass

        def sendTransform(self, t):
            STB.sent.append(t)
    tf.StaticTransformBroadcaster = STB
    mods["tf2_ros"] = tf
    sys.modules.update(mods)


def main():
    install_fakes()
    from ram_bug.ram_bug_node import RamBugNode
    from test_brain_offline import raycast
    from ram_bug import eg_core as eg

    node = RamBugNode()
    node.cfg["params_file"] = ""
    obst = {eg.idx(x, y) for x in range(10, 20) for y in range(12, 15)}     # one box in the middle
    node.brain.cfg.update(start_x=1.5, start_y=0.3, start_yaw=math.pi / 2)
    node.start_goal(1.5, 2.6)
    # robot sits still at its start for 3 planning cycles; check it wants to drive
    for k in range(3):
        node.get_clock().t = 0.5 * k
        odom = Odometry()
        node.subs["odom"](odom)
        x, y, yaw = node.pose()
        sc = LaserScan(ranges=[raycast(x, y, yaw + math.radians(i), obst, 9, 9) for i in range(360)],
                       angle_min=0.0, angle_increment=math.radians(1.0), range_min=0.12, range_max=3.5)
        node.subs["scan"](sc)
        node.on_plan()
        node.on_control()
    cmd = node.pubs["cmd_vel"].msgs[-1]
    print("mode:", node.pubs["ram_bug/mode"].msgs[-1].data, " target:", node.brain.target)
    print("cmd_vel: v = %.3f  w = %.3f" % (cmd.twist.linear.x, cmd.twist.angular.z))
    known = len(node.brain.planner.raw)
    print("obstacle cells seen:", known, " map msg cells:", sum(1 for d in node.pubs["ram_bug/map"].msgs[-1].data if d))
    assert known >= 8, "LiDAR did not map the box"
    assert node.brain.target is not None, "no target"
    print("OK")

    # ---- auto_start: robot at (2.1, 0.7) facing 120 deg, nobody tells it where it is
    RamBugNode.PARAMS = dict(RamBugNode.PARAMS, auto_start=True)
    node = RamBugNode()
    tx, ty, tyaw = 2.1, 0.7, math.radians(120)
    node.subs["goal_pose"](PoseStamped(header=Header(frame_id="arena"), pose=Pose(position=Msg(x=0.5, y=2.5, z=0.0))))
    for k in range(6):
        node.get_clock().t = 0.2 * k
        node.subs["odom"](Odometry())
        sc = LaserScan(ranges=[raycast(tx - 0.032 * math.cos(tyaw), ty - 0.032 * math.sin(tyaw),
                                       tyaw + math.radians(i), obst, 9, 9) for i in range(360)],
                       angle_min=0.0, angle_increment=math.radians(1.0), range_min=0.12, range_max=3.5)
        node.subs["scan"](sc)
    assert node.localized, "auto_start did not localize"
    x, y, yaw = node.pose()
    cands = [(tx, ty), (3 - ty, tx), (3 - tx, 3 - ty), (ty, 3 - tx)]
    err = min(math.hypot(x - a, y - b) for a, b in cands)
    print(f"auto_start pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)  err {err:.3f} m  goal {node.brain.goal}")
    assert err < 0.08 and node.brain.goal is not None
    print("AUTO OK")


if __name__ == "__main__":
    main()
