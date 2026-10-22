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


def run_round(seed, chaser_kind, cfg, tuned, verbose=False, share=None):
    """chaser_kind: Friend (the other team: shared positions, intercept / chase),
    Trap (waits for the particle trap) or Pursuit (straight chase)."""
    rng = random.Random(seed)
    friend = chaser_kind == "Friend"
    share = friend if share is None else share
    obst, start, goal, cst = eg.random_round(rng, "friend" if friend else "trap")
    sx, sy = [v * CELL for v in eg.center(start)]
    syaw = rng.uniform(-math.pi, math.pi)
    gx, gy = [v * CELL for v in eg.center(goal)]
    cx, cy = [v * CELL for v in eg.center(cst)]
    cyaw = math.atan2(sy - cy, sx - cx)          # the friend's robot starts facing us
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
    triggered = chaser_kind in ("Pursuit", "Friend")
    field, field_cell = None, None
    us_prev, us_v, close = None, (0.0, 0.0), False
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
            if share:                                   # UDP position from the chaser (+-2 cm noise)
                brain.set_peer(cx + rng.gauss(0, 0.02), cy + rng.gauss(0, 0.02), t)
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
            gx_, gy_ = x, y
            if friend:                                  # intercept far away, chase when close
                if us_prev is not None:
                    vx, vy = (x - us_prev[0]) / dt, (y - us_prev[1]) / dt
                    us_v = (0.9 * us_v[0] + 0.1 * vx, 0.9 * us_v[1] + 0.1 * vy)
                us_prev = (x, y)
                dd = math.hypot(x - cx, y - cy)
                close = True if dd < 0.8 else (False if dd > 1.0 else close)
                vx, vy = us_v if math.hypot(*us_v) >= 0.10 else (0.0, 0.0)
                tl = 0.2 if close else dd / 0.2
                gx_ = min(ARENA - 0.05, max(0.05, x + vx * tl))
                gy_ = min(ARENA - 0.05, max(0.05, y + vy * tl))
                if eg.cell_of((gx_ / CELL, gy_ / CELL)) in obst:
                    gx_, gy_ = x, y
            tcell = eg.cell_of((gx_ / CELL, gy_ / CELL))
            if step % 10 == 0 or field is None or tcell != field_cell:
                field, field_cell = eg.dijkstra(tcell, obst), tcell
            f = field
            cc = eg.cell_of((cx / CELL, cy / CELL))
            nb = min(eg.neighbours(cc, obst), key=lambda n: f[n])
            tx, ty = [vv * CELL for vv in eg.center(nb)]
            if nb == cc or math.hypot(x - cx, y - cy) < 0.25:
                tx, ty = (x, y) if math.hypot(x - cx, y - cy) < 0.25 else (gx_, gy_)
            d = math.hypot(tx - cx, ty - cy)
            if friend and d > 1e-6:                     # real TB3: has to turn before it drives
                err = math.atan2(ty - cy, tx - cx) - cyaw
                err = math.atan2(math.sin(err), math.cos(err))
                cyaw += max(-2.5 * dt, min(2.5 * dt, 3.0 * err))
                step_len = min(0.2 * dt, d) * max(0.0, math.cos(err))
                nx_, ny_ = cx + step_len * math.cos(cyaw), cy + step_len * math.sin(cyaw)
                if eg.cell_of((nx_ / CELL, ny_ / CELL)) not in obst:
                    cx, cy = nx_, ny_
            elif d > 1e-6:
                s = min(0.2 * dt, d) / d
                cx, cy = cx + (tx - cx) * s, cy + (ty - cy) * s
        if math.hypot(x - cx, y - cy) <= cfg["capture_r"]:
            return "caught", t
    return "timeout", t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rounds", type=int, default=20)
    ap.add_argument("--chaser", default="both", choices=["both", "Friend", "Trap", "Pursuit"])
    ap.add_argument("--no-share", action="store_true", help="Friend without position sharing (LiDAR only)")
    ap.add_argument("--margin", type=float, default=None, help="override plan_margin (m)")
    a = ap.parse_args()
    cfg = dict(capture_r=0.30, max_lin=0.20, max_ang=2.5, k_ang=3.0, lookahead_cells=3, inflate_cells=1,
               robot_max_cells=4, robot_max_extent=3, motion_min_free=2, time_limit=75.0,
               laser_x_offset=-0.032, max_free_range=3.0, stop_dist=0.13, plan_margin=0.0)
    if a.margin is not None:
        cfg["plan_margin"] = a.margin
    with open(os.path.join(HERE, "..", "config", "best_params.json"), encoding="utf-8") as f:
        tuned = json.load(f)["params"]
    kinds = ["Friend", "Pursuit"] if a.chaser == "both" else [a.chaser]
    for kind in kinds:
        res = {"reached": 0, "caught": 0, "timeout": 0}
        for s in range(a.rounds):
            o, t = run_round(30_000 + s, kind, cfg, tuned, share=False if a.no_share else None)
            res[o] += 1
            print(f"  {kind:8s} round {s:2d}: {o:8s} {t:5.1f} s", flush=True)
        n = sum(res.values())
        print(f"{kind}: reached {res['reached']}/{n}, caught {res['caught']}, timeout {res['timeout']}\n")


if __name__ == "__main__":
    main()
