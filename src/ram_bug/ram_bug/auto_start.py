"""
auto_start.py - find the arena walls in LiDAR scans -> where we start (no ROS imports)
====================================================================================
So nobody has to measure start_x / start_y / start_yaw: the robot stands still for
a moment, we collect a few scans and fit the square arena to them.

  1. direction : the walls are straight lines at 90 deg to each other. The direction of
                 every short segment between neighbouring LiDAR points, folded modulo
                 90 deg, piles up at the wall direction theta.
  2. position  : rotate all points by -theta; the walls are now vertical / horizontal
                 lines = tall peaks in the histograms of x and of y. Take the outermost
                 strong peak on each side (boxes inside the arena make smaller peaks,
                 things seen through gaps behind the walls are sparse).
  3. arena frame: origin = the corner (x_min, y_min), x axis along theta.
                 arena = R(-theta) * odom - (x_min, y_min)
                 => start_yaw = -theta, start = R(-theta) * (0, 0) - (x_min, y_min)
The square looks the same from all 4 sides, so the corner/axis are chosen as
"the wall direction closest to where the robot faces" and "the corner behind-right
of the robot". That is fine when the goal is clicked in RViz (it is converted with
the same frame).
"""
import math


def wall_direction(points, max_gap=0.06):
    """points: list of (x, y) in ONE scan order. Returns theta in [-pi/4, pi/4)."""
    sx = sy = 0.0
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        dx, dy = x2 - x1, y2 - y1
        d = math.hypot(dx, dy)
        if 0.005 < d < max_gap:
            a = 4.0 * math.atan2(dy, dx)          # fold modulo 90 deg
            sx += d * math.cos(a)
            sy += d * math.sin(a)
    if sx == 0.0 and sy == 0.0:
        return None
    th = math.atan2(sy, sx) / 4.0
    return (th + math.pi / 4) % (math.pi / 2) - math.pi / 4


def _peaks(vals, bin_w=0.03):
    """Histogram peaks: list of (position, count), sorted by position."""
    if not vals:
        return []
    lo = min(vals)
    n = int((max(vals) - lo) / bin_w) + 1
    h = [0] * n
    for v in vals:
        h[int((v - lo) / bin_w)] += 1
    out = []
    for i in range(n):
        c = h[i] + (h[i - 1] if i > 0 else 0) + (h[i + 1] if i + 1 < n else 0)
        if h[i] and h[i] >= (h[i - 1] if i > 0 else 0) and h[i] >= (h[i + 1] if i + 1 < n else 0):
            out.append((lo + (i + 0.5) * bin_w, c))
    return out


def _walls(vals, size, tol):
    """Pick (low, high) wall positions: strong peaks about `size` apart, outermost first."""
    pk = _peaks(vals)
    if not pk:
        return None
    strong = max(c for _, c in pk)
    cand = [p for p in pk if p[1] >= 0.25 * strong]
    best = None
    for lo_p, lo_c in cand:
        for hi_p, hi_c in cand:
            sep = hi_p - lo_p
            if abs(sep - size) <= tol:
                score = lo_c + hi_c
                if best is None or score > best[0]:
                    best = (score, lo_p, hi_p)
    if best is not None:
        return best[1], best[2]
    # only one wall seen well: use it and assume the arena size
    p, _ = max(cand, key=lambda q: q[1])
    mid = sum(vals) / len(vals)
    return (p, p + size) if p < mid else (p - size, p)


def fit_arena(scans, size=3.0, tol=0.35):
    """scans: list of point lists (odom frame, each in scan order).
    Returns dict(start_x, start_y, start_yaw, theta, walls, n) or None."""
    thetas = [t for t in (wall_direction(s) for s in scans) if t is not None]
    if not thetas:
        return None
    # average theta modulo 90 deg
    sx = sum(math.cos(4 * t) for t in thetas)
    sy = sum(math.sin(4 * t) for t in thetas)
    th = math.atan2(sy, sx) / 4.0
    c, s = math.cos(-th), math.sin(-th)
    xs, ys = [], []
    for sc in scans:
        for x, y in sc:
            xs.append(c * x - s * y)
            ys.append(s * x + c * y)
    if len(xs) < 50:
        return None
    wx, wy = _walls(xs, size, tol), _walls(ys, size, tol)
    if wx is None or wy is None:
        return None
    ox, oy = wx[0], wy[0]
    # odom origin (0, 0) rotated is still (0, 0) -> start = -(ox, oy)
    return dict(start_x=-ox, start_y=-oy, start_yaw=-th, theta=th,
                walls=(wx[0], wx[1], wy[0], wy[1]), n=len(xs))


def scan_points(pose, ranges, angle_min, angle_inc, range_min, range_max, laser_x=-0.032):
    """LaserScan -> list of (x, y) in the odom frame, in scan order."""
    x, y, yaw = pose
    lx, ly = x + laser_x * math.cos(yaw), y + laser_x * math.sin(yaw)
    out = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or not (range_min < r < range_max):
            continue
        a = yaw + angle_min + i * angle_inc
        out.append((lx + r * math.cos(a), ly + r * math.sin(a)))
    return out
