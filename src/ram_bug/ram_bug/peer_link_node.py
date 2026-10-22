"""
peer_link_node - position sharing between the two robots (UDP <-> ROS 2)
=======================================================================
Runs on OUR Raspberry Pi next to ram_bug_node. No password or SSH access is needed
by the other team: they only send/receive small UDP messages (see PROTOCOL_TH.md).

  /odom  ->  our arena pose  ->  UDP to <peer_host>:<send_port>        (10 Hz)
  UDP on <listen_port> (or the file /tmp/tb3_peer.json)  ->  /ram_bug/peer_pose
                                                         ->  /ram_bug/peer_status
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .peer_link import PeerFrame, PeerLink, read_file
from .ram_bug_node import yaw_of


class PeerLinkNode(Node):
    PARAMS = dict(
        peer_host="",                 # IP of the other robot ("" = only listen)
        send_port=47001, listen_port=47002, rate=10.0,
        peer_frame="arena",           # arena | peer_odom
        peer_start_x=0.0, peer_start_y=0.0, peer_start_yaw=0.0,   # only for peer_frame=peer_odom
        start_x=0.15, start_y=0.15, start_yaw=0.0,                # OUR start pose (arena frame)
        peer_file="",                 # e.g. /tmp/tb3_peer.json written by the other team's script
        max_outside=0.5,              # ignore peer poses farther than this outside the 3x3 m arena
    )

    def __init__(self):
        super().__init__("peer_link")
        for k, v in self.PARAMS.items():
            self.declare_parameter(k, v)
        P = {k: self.get_parameter(k).value for k in self.PARAMS}
        self.P = P
        self.link = PeerLink(P["peer_host"], P["send_port"], P["listen_port"])
        self.frame = PeerFrame(P["peer_frame"], (P["peer_start_x"], P["peer_start_y"], P["peer_start_yaw"]))
        self.odom = None
        self.prev = None
        self.v = (0.0, 0.0)
        self.file_mtime = None
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "ram_bug/peer_pose", 10)
        self.status_pub = self.create_publisher(String, "ram_bug/peer_status", 10)
        self.create_timer(1.0 / P["rate"], self.tick)
        self.create_timer(1.0, self.status)
        self.get_logger().info(
            f"sending to {P['peer_host'] or '(nobody)'}:{P['send_port']}, listening on UDP {P['listen_port']}"
            + (f", reading {P['peer_file']}" if P["peer_file"] else ""))

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def our_arena_pose(self):
        sx, sy, syaw = self.P["start_x"], self.P["start_y"], self.P["start_yaw"]
        ox, oy, oyaw = self.odom
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + c * ox - s * oy, sy + s * ox + c * oy, syaw + oyaw

    def tick(self):
        now = time.time()
        if self.odom is not None:
            x, y, yaw = self.our_arena_pose()
            if self.prev is not None:
                dt = max(1e-3, now - self.prev[2])
                vx, vy = (x - self.prev[0]) / dt, (y - self.prev[1]) / dt
                self.v = (0.7 * self.v[0] + 0.3 * vx, 0.7 * self.v[1] + 0.3 * vy)
            self.prev = (x, y, now)
            self.link.send(x, y, yaw, self.v[0], self.v[1], odom=self.odom)
        m = self.link.poll()
        if m is None and self.P["peer_file"]:
            m = self.from_file()
        if m is not None:
            self.publish(m)

    def from_file(self):
        import os
        try:
            mt = os.path.getmtime(self.P["peer_file"])
        except OSError:
            return None
        if mt == self.file_mtime:
            return None
        self.file_mtime = mt
        m = read_file(self.P["peer_file"])
        if m is not None:
            self.link.last, self.link.last_t = m, time.time()
            self.link.n_rx += 1
        return m

    def publish(self, m):
        x, y, yaw = self.frame.to_arena(m["x"], m["y"], m["yaw"])
        lim = self.P["max_outside"]
        if not (-lim <= x <= 3.0 + lim and -lim <= y <= 3.0 + lim):
            self.get_logger().warn(f"peer pose ({x:.2f}, {y:.2f}) is outside the arena - wrong frame? ignored",
                                   throttle_duration_sec=2.0)
            return
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "arena"
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pose_pub.publish(ps)

    def status(self):
        age = self.link.age()
        if age is None:
            s = "no packets yet"
            if self.link.n_bad:
                s += f" ({self.link.n_bad} unreadable, last: {self.link.last_raw[:80]!r})"
        else:
            s = f"ok, last packet {age:.1f} s ago, received {self.link.n_rx}"
        self.status_pub.publish(String(data=s))


def main():
    rclpy.init()
    n = PeerLinkNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.link.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
