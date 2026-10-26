"""
The numpy / scipy fast paths must give exactly what the pure-Python code gives.

  python3 test/test_fast_paths.py
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from ram_bug import eg_core as eg          # noqa: E402
from ram_bug.brain import Brain            # noqa: E402


def test_dijkstra_scipy_matches_python():
    if eg._np is None:
        print("scipy not installed: only the pure-Python Dijkstra is used")
        return
    rng = random.Random(1)
    for _ in range(200):
        blocked = {rng.randrange(eg.N) for _ in range(rng.randint(0, 300))}
        srcs = [rng.randrange(eg.N) for _ in range(rng.randint(1, 9))]   # sources may be blocked too
        fast = eg._dijkstra_sp(srcs, frozenset(blocked))
        for s, d in zip(srcs, fast):
            assert d == eg._dijkstra_py(s, blocked), s


def test_scan_numpy_matches_loop():
    if eg._np is None:
        print("numpy not installed: only the loop is used")
        return
    rng = random.Random(2)
    brain = Brain(dict(laser_x_offset=-0.032, max_free_range=3.0))
    np_mod = eg._np
    for _ in range(100):
        x, y, yaw = rng.uniform(-0.2, 3.2), rng.uniform(-0.2, 3.2), rng.uniform(-math.pi, math.pi)
        ranges = [rng.choice([0.0, float("nan"), float("inf"), rng.uniform(0.0, 4.0)]) for _ in range(360)]
        scan = (ranges, rng.uniform(-math.pi, math.pi), math.radians(1.0), 0.12, 3.5)
        fast = brain.scan_to_cells(x, y, yaw, *scan)
        eg._np = None
        try:
            slow = brain.scan_to_cells(x, y, yaw, *scan)
        finally:
            eg._np = np_mod
        assert fast == slow, (x, y, yaw)


if __name__ == "__main__":
    test_dijkstra_scipy_matches_python()
    test_scan_numpy_matches_loop()
    print("FAST PATHS OK")
