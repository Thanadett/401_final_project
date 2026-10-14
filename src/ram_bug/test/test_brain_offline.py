"""
Offline test of the ROS brain WITHOUT ROS or Gazebo.

A small physics loop: unicycle robot (v, w) at 20 Hz, synthetic 360-beam LaserScan
(0.12-3.5 m, like the LDS-02), random obstacles, and a chaser that moves at the same
speed. This checks the parts the grid game does not: LaserScan -> grid conversion,
odom -> arena frame, and the heading controller.

  python3 test/test_brain_offline.py            # 20 rounds vs Trap and vs Pursuit
  python3 test/test_brain_offline.py --rounds 5 --chaser Trap
"""
import argparse
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from ram_bug import eg_core as eg          # noqa: E402
from ram_bug.brain import Brain, wrap      # noqa: E402

CELL = eg.CELL
ARENA = eg.W * CELL
R_CHASER = 0.07
R_ROBOT = 0.08


def raycast(x, y, a, obst, cx, cy, rmax=3.5):
    ca, sa = math.cos(a), math.sin(a)
    t = 0.12
    while t < rmax:
        px, py = x + t * ca, y + t * sa
        if not (0 <= px < ARENA and 0 <= py < ARENA):
            return t                                   # arena wall
        if eg.cell_of((px / CELL, py / CELL)) in obst:
            return t
        if (px - cx) ** 2 + (py - cy) ** 2 <= R_CHASER ** 2:
            return t
        t += 0.02
    return float("inf")


def free_at(x, y, obst, r):
    for dx in (-r, 0, r):
        for dy in (-r, 0, r):
            px, py = x + dx, y + dy
            if not (0 <= px < ARENA and 0 <= py < ARENA):
                return False
            if eg.cell_of((px / CELL, py / CELL)) in obst:
                return False
    return True


def run_round(seed, chaser_kind, cfg, tuned, verbose=False):
    rng = random.Random(seed)
    obst, start, goal, cst = eg.random_round(rng)
    sx, sy = [v * CELL for v in eg.center(start)]
    syaw = rng.uniform(-math.pi, math.pi)
    gx, gy = [v * CELL for v in eg.center(goal)]
    cx, cy = [v * CELL for v in eg.center(cst)]
    cfg = dict(cfg, start_x=sx, start_y=sy, start_yaw=syaw)
    brain = Brain(cfg, tuned)
    brain.set_goal(gx, gy)

    # odometry state (starts at 0,0,0 like the real robot)
    ox = oy = oyaw = 0.0
    particles = []
    while len(particles) < 40:
        a, r = rng.uniform(0, 2 * math.pi), 0.6 * math.sqrt(rng.random())
        p = (gx + r * math.cos(a), gy + r * math.sin(a))
        if 0 <= p[0] < ARENA and 0 <= p[1] < ARENA and eg.cell_of((p[0] / CELL, p[1] / CELL)) not in obst:
            particles.append(p)
    triggered = chaser_kind == "Pursuit"
    field = None
    dt = 0.05
    scan = None
    v = w = 0.0
    for step in range(int(80 / dt)):
        t = step * dt
        x, y, yaw = brain.odom_to_arena(ox, oy, oyaw)
        if step % 10 == 0:                              # 2 Hz LiDAR + planning
            ranges = [raycast(x - 0.032 * math.cos(yaw), y - 0.032 * math.sin(yaw),
                              yaw + math.radians(k), obst, cx, cy) for k in range(360)]
            scan = (ranges, 0.0, math.radians(1.0), 0.12, 3.5)
            brain.plan(x, y, yaw, scan, t)
            if brain.done:
                return brain.done, t
        front = min([r for k, r in enumerate(scan[0]) if k <= 30 or k >= 330] + [9.9])
        v, w = brain.control(x, y, yaw, front, t)
        # robot motion (in odom frame), blocked by obstacles
        nyaw = wrap(oyaw + w * dt)
        nox, noy = ox + v * dt * math.cos(oyaw), oy + v * dt * math.sin(oyaw)
        ax, ay, _ = brain.odom_to_arena(nox, noy, nyaw)
        if free_at(ax, ay, obst, R_ROBOT * 0.7):
            ox, oy = nox, noy
        oyaw = nyaw
        x, y, yaw = brain.odom_to_arena(ox, oy, oyaw)
        # chaser
        if not triggered and any(math.hypot(x - px, y - py) <= 0.15 for px, py in particles):
            triggered = True
        if triggered:
            if step % 10 == 0 or field is None:
                field = eg.dijkstra(eg.cell_of((x / CELL, y / CELL)), obst)
            f = field
            cc = eg.cell_of((cx / CELL, cy / CELL))
            nb = min(eg.neighbours(cc, obst), key=lambda n: f[n])
            tx, ty = [vv * CELL for vv in eg.center(nb)]
            if nb == cc or math.hypot(x - cx, y - cy) < 0.25:
                tx, ty = x, y
            d = math.hypot(tx - cx, ty - cy)
            if d > 1e-6:
                s = min(0.2 * dt, d) / d
                cx, cy = cx + (tx - cx) * s, cy + (ty - cy) * s
        if math.hypot(x - cx, y - cy) <= cfg["capture_r"]:
            return "caught", t
    return "timeout", t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rounds", type=int, default=20)
    ap.add_argument("--chaser", default="both", choices=["both", "Trap", "Pursuit"])
    a = ap.parse_args()
    cfg = dict(capture_r=0.30, max_lin=0.20, max_ang=2.5, k_ang=3.0, lookahead_cells=3, inflate_cells=1,
               robot_max_cells=4, robot_max_extent=3, motion_min_free=2, time_limit=75.0,
               laser_x_offset=-0.032, max_free_range=3.0, stop_dist=0.13)
    with open(os.path.join(HERE, "..", "config", "best_params.json"), encoding="utf-8") as f:
        tuned = json.load(f)["params"]
    kinds = ["Trap", "Pursuit"] if a.chaser == "both" else [a.chaser]
    for kind in kinds:
        res = {"reached": 0, "caught": 0, "timeout": 0}
        for s in range(a.rounds):
            o, t = run_round(30_000 + s, kind, cfg, tuned)
            res[o] += 1
            print(f"  {kind:8s} round {s:2d}: {o:8s} {t:5.1f} s", flush=True)
        n = sum(res.values())
        print(f"{kind}: reached {res['reached']}/{n}, caught {res['caught']}, timeout {res['timeout']}\n")


if __name__ == "__main__":
    main()
