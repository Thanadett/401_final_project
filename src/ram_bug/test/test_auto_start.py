"""Synthetic test of auto_start.fit_arena: square arena with gaps, boxes, noise,
things behind the walls, random robot pose.   python3 test/test_auto_start.py"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from ram_bug.auto_start import fit_arena, scan_points   # noqa: E402


def segs_for(rng):
    S = 3.0
    walls = [((0, 0), (S, 0)), ((S, 0), (S, S)), ((S, S), (0, S)), ((0, S), (0, 0))]
    segs = []
    for a, b in walls:                       # walls with 1-2 gaps
        t = sorted(rng.uniform(0.1, 0.9) for _ in range(2))
        g = 0.25
        for u0, u1 in ((0, t[0]), (t[0] + g / S, t[1]), (t[1] + g / S, 1)):
            if u1 > u0:
                segs.append(((a[0] + (b[0] - a[0]) * u0, a[1] + (b[1] - a[1]) * u0),
                             (a[0] + (b[0] - a[0]) * u1, a[1] + (b[1] - a[1]) * u1)))
    for _ in range(4):                       # boxes 0.2-0.4 m
        cx, cy, w = rng.uniform(0.6, 2.4), rng.uniform(0.6, 2.4), rng.uniform(0.2, 0.4)
        p = [(cx - w / 2, cy - w / 2), (cx + w / 2, cy - w / 2), (cx + w / 2, cy + w / 2), (cx - w / 2, cy + w / 2)]
        segs += list(zip(p, p[1:] + p[:1]))
    segs.append(((-0.8, -1.0), (4.0, -1.2)))  # a room wall behind the arena
    return segs


def ray(px, py, a, segs):
    best = None
    dx, dy = math.cos(a), math.sin(a)
    for (x1, y1), (x2, y2) in segs:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - px) * ey - (y1 - py) * ex) / den
        u = ((x1 - px) * dy - (y1 - py) * dx) / den
        if t > 0 and 0 <= u <= 1 and (best is None or t < best):
            best = t
    return best


def main():
    ok = 0
    for k in range(30):
        rng = random.Random(k)
        segs = segs_for(rng)
        rx, ry, ryaw = rng.uniform(0.3, 2.7), rng.uniform(0.3, 2.7), rng.uniform(-math.pi, math.pi)
        if any(min(x1, x2) - 0.25 < rx < max(x1, x2) + 0.25 and min(y1, y2) - 0.25 < ry < max(y1, y2) + 0.25
               for (x1, y1), (x2, y2) in segs[12:-1]):
            rx, ry = 0.3, 0.3
        scans = []
        for _ in range(5):
            rngs = []
            for i in range(360):
                r = ray(rx - 0.032 * math.cos(ryaw), ry - 0.032 * math.sin(ryaw), ryaw + math.radians(i), segs)
                rngs.append(float("inf") if r is None or r > 3.5 else r + rng.gauss(0, 0.01))
            scans.append(scan_points((0.0, 0.0, 0.0), rngs, 0.0, math.radians(1), 0.12, 3.5))
        f = fit_arena(scans)
        assert f is not None, k
        # true pose expressed in the fitted frame must be one of the 4 symmetric versions
        # of (rx, ry, ryaw): check that walls map onto 0 / 3 by testing the robot pose itself
        sx, sy, syaw = f["start_x"], f["start_y"], f["start_yaw"]
        cands = [(rx, ry), (3 - ry, rx), (3 - rx, 3 - ry), (ry, 3 - rx)]   # rotations of the square
        err = min(math.hypot(sx - a, sy - b) for a, b in cands)
        ok += err < 0.06
        print(f"case {k:2d}: true ({rx:.2f},{ry:.2f},{math.degrees(ryaw):6.1f})  "
              f"fit ({sx:.2f},{sy:.2f},{math.degrees(syaw):6.1f})  err {err:.3f}")
    print(f"{ok}/30 within 6 cm")
    assert ok >= 28
    print("ALL OK")


if __name__ == "__main__":
    main()
