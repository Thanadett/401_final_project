#!/usr/bin/env python3
"""
tb3_connect.py - check the position-sharing link WITHOUT ROS and WITHOUT SSH
===========================================================================
Replaces the SSH-based tb3_connect.py: nobody needs the other robot's password.
Each team runs its own sender/receiver on its own Pi (see PROTOCOL_TH.md).

On OUR Pi (evader):
    python3 tools/tb3_connect.py --peer 192.168.1.60            # IP of the chaser's Pi
It sends a fixed test pose to <peer>:47001 at 10 Hz and prints every pose received on 47002.

Simulate the chaser on a laptop (to test our side before the match):
    python3 tools/tb3_connect.py --peer <our Pi IP> --role pursuer --x 1.5 --y 1.5

Stop the ram_bug launch first (both use UDP 47002).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from ram_bug.peer_link import PeerLink   # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--peer", required=True, help="IP address of the other robot")
    ap.add_argument("--role", default="evader", choices=["evader", "pursuer"])
    ap.add_argument("--x", type=float, default=0.3)
    ap.add_argument("--y", type=float, default=0.3)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=0, help="0 = run until Ctrl+C")
    a = ap.parse_args()
    send, listen = (47001, 47002) if a.role == "evader" else (47002, 47001)
    link = PeerLink(a.peer, send_port=send, listen_port=listen, team=a.role)
    print(f"[{a.role}] sending ({a.x}, {a.y}) to {a.peer}:{send}, listening on UDP {listen}  (Ctrl+C to stop)")
    t0, last_print = time.time(), 0.0
    try:
        while not a.seconds or time.time() - t0 < a.seconds:
            link.send(a.x, a.y, a.yaw)
            m = link.poll()
            now = time.time()
            if m is not None and now - last_print > 0.5:
                print(f"  received from {m['team'] or '?'}: x={m['x']:.2f} y={m['y']:.2f} yaw={m['yaw']:.2f}"
                      f"  frame={m['frame'] or '?'}  (total {link.n_rx})")
                last_print = now
            elif m is None and now - last_print > 2.0:
                age = link.age()
                print("  nothing received yet" if age is None else f"  no packet for {age:.1f} s",
                      f"(unreadable: {link.n_bad}, last raw: {link.last_raw[:60]!r})" if link.n_bad else "")
                last_print = now
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    print(f"done: received {link.n_rx} good, {link.n_bad} unreadable")


if __name__ == "__main__":
    main()
