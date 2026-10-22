"""
peer_link.py - share positions with the other robot over UDP (no ROS imports)
=============================================================================
Both teams agreed to share positions over Wi-Fi. This module is the transport:

  WE SEND (10 Hz) one JSON object per UDP datagram to <peer_host>:<send_port>:
    {"team": "evader", "seq": 17, "t": 1760000000.12,
     "frame": "arena", "x": 1.52, "y": 0.84, "yaw": 1.57, "vx": 0.18, "vy": 0.02,
     "odom_x": 0.70, "odom_y": 0.12, "odom_yaw": 0.0}
    x, y, yaw  : our pose in the ARENA frame (metres / radians, origin = arena corner)
    odom_*     : the same pose in our own /odom frame (for teams that align frames themselves)

  WE RECEIVE on 0.0.0.0:<listen_port>. Accepted formats (first that matches):
    JSON  {"x": .., "y": .., "yaw"|"theta"|"heading": .., "frame": ..}
    JSON  {"pose": {"x": .., "y": .., "yaw": ..}}   or {"position": [x, y], ...}
    text  "x,y,yaw"  or  "x y yaw"
  The peer's pose is converted to the arena frame by PeerFrame (see below).

Compatibility: the other team's script can also write the chaser pose to a file on our
Pi (/tmp/tb3_peer.json); read_file() parses that file with the same rules.
"""
import json
import math
import socket
import time


def _num(d, *keys):
    for k in keys:
        if k in d and d[k] is not None:
            try:
                return float(d[k])
            except (TypeError, ValueError):
                pass
    return None


def parse_message(data):
    """bytes/str -> dict(x, y, yaw, frame, team) or None."""
    if isinstance(data, (bytes, bytearray)):
        data = data.decode("utf-8", errors="ignore")
    data = data.strip()
    if not data:
        return None
    try:
        d = json.loads(data)
    except ValueError:
        d = None
    if isinstance(d, dict):
        src = d
        for key in ("pose", "pursuer", "chaser", "peer", "robot"):
            if isinstance(d.get(key), dict):
                src = d[key]
                break
        x, y = _num(src, "x", "px", "pos_x"), _num(src, "y", "py", "pos_y")
        if (x is None or y is None) and isinstance(src.get("position"), (list, tuple)) and len(src["position"]) >= 2:
            x, y = float(src["position"][0]), float(src["position"][1])
        if (x is None or y is None) and isinstance(src.get("position"), dict):
            x, y = _num(src["position"], "x"), _num(src["position"], "y")
        if x is None or y is None:
            return None
        yaw = _num(src, "yaw", "theta", "heading", "th")
        return {"x": x, "y": y, "yaw": yaw if yaw is not None else 0.0,
                "frame": str(d.get("frame", src.get("frame", ""))), "team": str(d.get("team", ""))}
    parts = data.replace(",", " ").split()
    try:
        vals = [float(p) for p in parts[:3]]
    except ValueError:
        return None
    if len(vals) < 2:
        return None
    return {"x": vals[0], "y": vals[1], "yaw": vals[2] if len(vals) > 2 else 0.0, "frame": "", "team": ""}


def read_file(path):
    try:
        with open(path, encoding="utf-8") as f:
            return parse_message(f.read())
    except OSError:
        return None


class PeerFrame:
    """Convert the peer's reported pose into OUR arena frame.
       mode "arena"     : peer already sends arena coordinates (recommended, both teams
                          measure from the same arena corner)
       mode "peer_odom" : peer sends its own odometry; we add its start pose
                          (peer_start_x/y/yaw, measured in the arena frame)"""

    def __init__(self, mode="arena", start=(0.0, 0.0, 0.0)):
        self.mode = mode
        self.start = start

    def to_arena(self, x, y, yaw):
        if self.mode == "peer_odom":
            sx, sy, syaw = self.start
            c, s = math.cos(syaw), math.sin(syaw)
            return sx + c * x - s * y, sy + s * x + c * y, syaw + yaw
        return x, y, yaw


class PeerLink:
    def __init__(self, peer_host, send_port=47001, listen_port=47002, team="evader"):
        self.peer = (peer_host, int(send_port)) if peer_host else None
        self.team = team
        self.seq = 0
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", int(listen_port)))
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last = None              # last parsed message
        self.last_t = None            # local receive time
        self.n_rx = 0
        self.n_bad = 0
        self.last_raw = b""

    def send(self, x, y, yaw, vx=0.0, vy=0.0, odom=None):
        if self.peer is None:
            return False
        self.seq += 1
        msg = {"team": self.team, "seq": self.seq, "t": round(time.time(), 3), "frame": "arena",
               "x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4),
               "vx": round(vx, 4), "vy": round(vy, 4)}
        if odom is not None:
            msg.update(odom_x=round(odom[0], 4), odom_y=round(odom[1], 4), odom_yaw=round(odom[2], 4))
        try:
            self.tx.sendto(json.dumps(msg).encode(), self.peer)
            return True
        except OSError:
            return False

    def poll(self):
        """Read every waiting datagram; keep the newest valid one. Returns it or None."""
        got = None
        while True:
            try:
                data, _ = self.rx.recvfrom(4096)
            except (BlockingIOError, OSError):
                break
            self.last_raw = data
            m = parse_message(data)
            if m is None:
                self.n_bad += 1
                continue
            got = m
        if got is not None:
            self.last, self.last_t = got, time.time()
            self.n_rx += 1
        return got

    def age(self):
        return None if self.last_t is None else time.time() - self.last_t

    def close(self):
        for s in (self.rx, self.tx):
            try:
                s.close()
            except OSError:
                pass
