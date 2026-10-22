"""
Test the UDP position-sharing code WITHOUT ROS (loopback on this computer).

  python3 test/test_peer_link.py
"""
import json
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from ram_bug.peer_link import PeerFrame, PeerLink, parse_message   # noqa: E402


def main():
    # 1) message formats the other team might use
    cases = {
        '{"x": 1.2, "y": 0.5, "yaw": 0.3}': (1.2, 0.5, 0.3),
        '{"x": 1.2, "y": 0.5, "theta": 0.3, "frame": "arena"}': (1.2, 0.5, 0.3),
        '{"pose": {"x": 2.0, "y": 1.0, "yaw": 1.0}, "team": "pursuer"}': (2.0, 1.0, 1.0),
        '{"position": [0.4, 0.6], "heading": -1.0}': (0.4, 0.6, -1.0),
        '{"pursuer": {"px": 0.1, "py": 0.2}}': (0.1, 0.2, 0.0),
        "1.5,2.5,0.7": (1.5, 2.5, 0.7),
        "1.5 2.5": (1.5, 2.5, 0.0),
    }
    for raw, want in cases.items():
        m = parse_message(raw.encode())
        got = (m["x"], m["y"], m["yaw"])
        assert all(abs(a - b) < 1e-9 for a, b in zip(got, want)), (raw, got)
    assert parse_message(b"hello") is None and parse_message(b'{"a": 1}') is None
    print("formats: OK")

    # 2) frame conversion: peer sends its odometry, it started at (2.5, 0.5) facing +y
    f = PeerFrame("peer_odom", (2.5, 0.5, math.pi / 2))
    x, y, yaw = f.to_arena(1.0, 0.0, 0.0)          # 1 m forward = +y in the arena
    assert abs(x - 2.5) < 1e-9 and abs(y - 1.5) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9
    print("frames: OK")

    # 3) real UDP on loopback: two links talking to each other
    a = PeerLink("127.0.0.1", send_port=47912, listen_port=47911, team="evader")
    b = PeerLink("127.0.0.1", send_port=47911, listen_port=47912, team="pursuer")
    for k in range(5):
        a.send(0.5 + 0.1 * k, 1.0, 0.0, 0.2, 0.0, odom=(0.1 * k, 0.0, 0.0))
        time.sleep(0.01)
    time.sleep(0.05)
    m = b.poll()
    assert m is not None and abs(m["x"] - 0.9) < 1e-6, m      # newest message wins
    raw = json.loads(b.last_raw)
    assert raw["team"] == "evader" and raw["frame"] == "arena" and "odom_x" in raw
    b.send(2.0, 2.0, 1.0)
    time.sleep(0.05)
    m = a.poll()
    assert m is not None and m["x"] == 2.0
    # garbage is counted, not crashing
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(b"\xff\xfe junk", ("127.0.0.1", 47911))
    time.sleep(0.05)
    assert a.poll() is None and a.n_bad == 1
    a.close(), b.close(), s.close()
    print("udp loopback: OK")
    print("ALL OK")


if __name__ == "__main__":
    main()
