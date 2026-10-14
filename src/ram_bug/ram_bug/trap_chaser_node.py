"""
trap_chaser_node - a TEST opponent for Gazebo (not used on the real robot)
=========================================================================
Behaves like the other team described: scatters 'particles' around the goal and
waits; when our robot passes within `detect_r` of a particle it chases us at
`speed` along the shortest path (it knows the obstacles from the world json).

The chaser in the world file is a red cylinder driven by gz VelocityControl
(topic /model/chaser/cmd_vel) with an OdometryPublisher (topic /chaser/odom,
world = arena frame).
"""
import json
import math
import random

import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from . import eg_core as eg
from .ram_bug_node import yaw_of


class TrapChaser(Node):
    def __init__(self):
        super().__init__("trap_chaser")
        defaults = dict(world_json="", goal_x=1.5, goal_y=2.8, start_x=0.15, start_y=0.15,
                        start_yaw=0.0, particle_r=0.6, n_particles=40, detect_r=0.15,
                        speed=0.20, capture_r=0.30, mode="trap", seed=1)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        P = {k: self.get_parameter(k).value for k in defaults}
        self.P = P
        self.obst = set()
        if P["world_json"]:
            with open(P["world_json"], encoding="utf-8") as f:
                w = json.load(f)
            self.obst = set(w["obstacle_cells"])
        rng = random.Random(P["seed"])
        gx, gy = P["goal_x"], P["goal_y"]
        self.particles = []
        while len(self.particles) < P["n_particles"]:
            a, r = rng.uniform(0, 2 * math.pi), P["particle_r"] * math.sqrt(rng.random())
            p = (gx + r * math.cos(a), gy + r * math.sin(a))
            if 0 <= p[0] < 3 and 0 <= p[1] < 3 and eg.cell_of((p[0] / 0.1, p[1] / 0.1)) not in self.obst:
                self.particles.append(p)
        self.triggered = P["mode"] == "pursuit"
        self.me = None
        self.us = None
        self.over = False
        self.create_subscription(Odometry, "/chaser/odom", self.on_me, 10)
        self.create_subscription(Odometry, "/odom", self.on_us, 10)
        self.pub = self.create_publisher(Twist, "/model/chaser/cmd_vel", 10)
        self.part_pub = self.create_publisher(PoseArray, "/chaser/particles", 1)
        self.state_pub = self.create_publisher(String, "/chaser/state", 10)
        self.create_timer(0.05, self.step)
        self.create_timer(1.0, self.publish_particles)
        self.field, self.field_cell = None, None

    def on_me(self, msg):
        p = msg.pose.pose
        self.me = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_us(self, msg):
        p = msg.pose.pose
        sx, sy, syaw = self.P["start_x"], self.P["start_y"], self.P["start_yaw"]
        c, s = math.cos(syaw), math.sin(syaw)
        ox, oy = p.position.x, p.position.y
        self.us = (sx + c * ox - s * oy, sy + s * ox + c * oy)

    def publish_particles(self):
        pa = PoseArray()
        pa.header.frame_id = "arena"
        pa.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.particles:
            p = Pose()
            p.position.x, p.position.y = x, y
            pa.poses.append(p)
        self.part_pub.publish(pa)

    def step(self):
        if self.me is None or self.us is None or self.over:
            return
        x, y, yaw = self.me
        ux, uy = self.us
        if math.hypot(ux - x, uy - y) <= self.P["capture_r"]:
            self.over = True
            self.pub.publish(Twist())
            self.state_pub.publish(String(data="CAUGHT"))
            self.get_logger().info("CAUGHT the evader!")
            return
        if not self.triggered:
            if any(math.hypot(ux - px, uy - py) <= self.P["detect_r"] for px, py in self.particles):
                self.triggered = True
                self.get_logger().info("trap triggered - chasing")
            else:
                self.pub.publish(Twist())
                self.state_pub.publish(String(data="WAITING"))
                return
        # shortest path toward the evader on the true map
        uc = eg.cell_of((ux / 0.1, uy / 0.1))
        if uc != self.field_cell:
            self.field, self.field_cell = eg.dijkstra(uc, self.obst), uc
        mc = eg.cell_of((x / 0.1, y / 0.1))
        nb = min(eg.neighbours(mc, self.obst), key=lambda n: self.field[n])
        tx, ty = (ux, uy) if nb == mc else tuple(v * 0.1 for v in eg.center(nb))
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        tw = Twist()
        if d > 1e-3:
            vx, vy = self.P["speed"] * dx / d, self.P["speed"] * dy / d
            c, s = math.cos(-yaw), math.sin(-yaw)              # world -> body frame
            tw.linear.x, tw.linear.y = c * vx - s * vy, s * vx + c * vy
        self.pub.publish(tw)
        self.state_pub.publish(String(data="CHASING"))


def main():
    rclpy.init()
    n = TrapChaser()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
