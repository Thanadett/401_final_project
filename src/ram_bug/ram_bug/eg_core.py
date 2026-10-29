"""
Goal-Reaching Evader - simulation core (headless, no pygame)
=============================================================
Competition task: OUR robot is the EVADER. It is given a GOAL point and must drive
there while avoiding the CHASER robot built by another team.

  arena      : 3 x 3 m, map grid 30 x 30 (1 cell = 0.10 m)
  motion     : continuous positions; both robots move at the SAME speed,
               0.10 m per tick (tick = 0.5 s -> 0.20 m/s), in any of 8 headings
  obstacles  : random boxes, different every round, NOT known in advance:
               our robot builds its map from a simulated 360-degree LiDAR
  caught     : centre distance <= capture_r   (default 0.30 m, adjustable)
  success    : evader within GOAL_R (0.15 m) of the goal before being caught
  timeout    : T ticks (default 150 = 75 s)

Evader algorithms
  RAM-Bug     : our algorithm (Race-Aware Adaptive Minimax Bug) - see ALGORITHM.md
  AM-Bug      : earlier algorithm (minimax lookahead + adaptive lambda only)
  RAM-noBayes / RAM-noRace / RAM-noTerr : RAM-Bug with one mechanism removed (ablation)
  A*-Replan   : drive the shortest known path to the goal, ignore the chaser
  APF         : artificial potential field (goal attracts, chaser repels)

Chaser opponents (we do not know what the other team will build, so test many)
  Pursuit     : shortest path straight at the evader
  Intercept   : guesses our goal from how we move (Bayes) and cuts us off
  Guard       : KNOWS our goal and blocks the way (worst case)
  Predict     : 1-step lookahead, assumes the evader runs away
  Random      : random walk
  Human       : keyboard (game.py)
Chasers know the true map. With chaser_sight="los" they only see us in line of sight.
"""
import heapq
import math
import os
import random

try:                       # optional: C Dijkstra, many sources per call (sudo apt install python3-scipy)
    if os.environ.get("RAM_BUG_PURE_PYTHON"):
        raise ImportError
    import numpy as _np
    from scipy.sparse import csr_matrix as _csr
    from scipy.sparse.csgraph import dijkstra as _sp_dijkstra
except ImportError:
    _np = None

W = H = 30
N = W * H
CELL = 0.10
TICK_S = 0.5
STEP = 1.0                 # cells per tick (= 0.10 m)
MOVES = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
INF = 1e9
SQ2 = math.sqrt(2.0)
GOAL_R = 1.5               # cells (0.15 m)
FAR_FROM_GOAL = 15         # chaser starts >= 1.5 m from the goal (competition info)
POINTS = {"reached": 3, "timeout": 1, "caught": 0}   # win / draw (time up, not caught) / loss
LIDAR_RANGE = 35           # cells = 3.5 m (TurtleBot3 LDS)
LIDAR_RAYS = 180


def idx(x, y):
    return y * W + x


def xy(i):
    return i % W, i // W


def center(i):
    x, y = xy(i)
    return (x + 0.5, y + 0.5)


def cell_of(p):
    return idx(min(W - 1, max(0, int(p[0]))), min(H - 1, max(0, int(p[1]))))


def dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def cdist(a, b):
    """distance between two cell centres"""
    ax, ay = xy(a)
    bx, by = xy(b)
    return math.hypot(ax - bx, ay - by)


def _neighbour_table():
    """Per cell, in MOVES order: (cell, step cost, side cell a, side cell b). A diagonal step
    is only allowed when both side cells are free (no corner cutting); straight steps use -1."""
    table = []
    for i in range(N):
        x, y = i % W, i // W
        row = []
        for dx, dy in MOVES[1:]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H:
                if dx and dy:
                    row.append((ny * W + nx, SQ2, idx(x + dx, y), idx(x, y + dy)))
                else:
                    row.append((ny * W + nx, 1.0, -1, -1))
        table.append(tuple(row))
    return tuple(table)


_NB = _neighbour_table()   # precomputed: these two run ~1e6 times per round on the robot


def _near_table():
    """Per cell: cells at Chebyshev distance <= 1, and 2..3 (inside the grid), same order as before."""
    table = []
    for i in range(N):
        cx, cy = i % W, i // W
        n1, n3 = [], []
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                x, y = cx + dx, cy + dy
                if 0 <= x < W and 0 <= y < H:
                    (n1 if max(abs(dx), abs(dy)) <= 1 else n3).append(y * W + x)
        table.append((tuple(n1), tuple(n3)))
    return tuple(table)


_NEAR13 = _near_table()    # for RAMBug.all_rings


def neighbours(i, blocked, stay=True):
    out = [i] if stay else []
    for j, _, a, b in _NB[i]:
        if j in blocked or a in blocked or b in blocked:
            continue
        out.append(j)
    return out


def dijkstra(src, blocked):
    """Travel time (ticks) from src to every cell; diagonal steps cost sqrt(2).
    The result is shared with a small cache: callers must not modify it."""
    return dijkstra_many((src,), blocked)[0]


_CACHE = {}                # (src, frozenset(blocked)) -> distances; the map often does not change


def dijkstra_many(srcs, blocked):
    """dijkstra() for several sources at once (one C call with scipy)."""
    key = frozenset(blocked)
    out = [_CACHE.get((s, key)) for s in srcs]
    todo = [s for s, d in zip(srcs, out) if d is None]
    if todo:
        fresh = _dijkstra_sp(todo, key) if _np is not None else [_dijkstra_py(s, key) for s in todo]
        got = dict(zip(todo, fresh))
        out = [got[s] if d is None else d for s, d in zip(srcs, out)]   # not from _CACHE: it may be cleared
        if len(_CACHE) > 64:
            _CACHE.clear()
        for s, d in got.items():
            _CACHE[(s, key)] = d
    return out


if _np is not None:
    # every grid step once: (u, v, cost, side a, side b); a straight step has no side cells (index N, never blocked)
    _EDGES = [(u, v, c, a if a >= 0 else N, b if b >= 0 else N) for u in range(N) for v, c, a, b in _NB[u]]
    _EU, _EV, _EC, _EA, _EB = (_np.array(col) for col in zip(*_EDGES))


def _dijkstra_sp(srcs, blocked):
    mask = _np.zeros(N + 1, dtype=bool)
    if blocked:
        mask[_np.fromiter(blocked, dtype=_np.int64, count=len(blocked))] = True
    ok = ~(mask[_EV] | mask[_EA] | mask[_EB])         # same rule as neighbours(): v and both sides free
    g = _csr((_EC[ok], (_EU[ok], _EV[ok])), shape=(N, N))
    d = _sp_dijkstra(g, directed=True, indices=list(srcs))
    d[_np.isinf(d)] = INF
    return d.tolist()


def _dijkstra_py(src, blocked):
    d = [INF] * N
    d[src] = 0.0
    pq = [(0.0, src)]
    pop, push, nb = heapq.heappop, heapq.heappush, _NB
    while pq:
        du, u = pop(pq)
        if du > d[u]:
            continue
        for v, c, a, b in nb[u]:
            if v in blocked or a in blocked or b in blocked:
                continue
            nd = du + c
            if nd < d[v]:
                d[v] = nd
                push(pq, (nd, v))
    return d


def advance(p, target_cell, obst):
    """Move point p one STEP toward the centre of target_cell (blocked -> slide / stay)."""
    tx, ty = center(target_cell)
    dx, dy = tx - p[0], ty - p[1]
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return p
    s = min(STEP, L) / L
    cand = [(p[0] + dx * s, p[1] + dy * s)]
    if abs(dx) > 1e-9:
        cand.append((p[0] + math.copysign(min(STEP, abs(dx)), dx), p[1]))
    if abs(dy) > 1e-9:
        cand.append((p[0], p[1] + math.copysign(min(STEP, abs(dy)), dy)))
    for q in cand:
        if 0 <= q[0] < W and 0 <= q[1] < H and cell_of(q) not in obst:
            return q
    return p


def line_cells(a, b):
    x0, y0 = xy(a)
    x1, y1 = xy(b)
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    out = []
    while True:
        out.append(idx(x0, y0))
        if x0 == x1 and y0 == y1:
            return out
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def line_of_sight(a, b, obst):
    return not any(c in obst for c in line_cells(a, b))


_RAYS = []
for _k in range(LIDAR_RAYS):
    _ang = 2 * math.pi * _k / LIDAR_RAYS
    _dx, _dy = math.cos(_ang), math.sin(_ang)
    _seq, _seen = [], set()
    _t = 0.5
    while _t <= LIDAR_RANGE:
        _o = (int(math.floor(0.5 + _dx * _t)), int(math.floor(0.5 + _dy * _t)))
        if _o not in _seen and _o != (0, 0):
            _seen.add(_o)
            _seq.append(_o)
        _t += 0.5
    _RAYS.append(_seq)


def lidar_scan(pos_cell, obst, other_cell):
    """Simulated LiDAR. Like a real scan it returns UNLABELLED hit cells: the other
    robot is just another return. Also returns the cells each ray passed through
    (seen free) and the ray end points for drawing."""
    x0, y0 = xy(pos_cell)
    hits, free, ends = set(), set(), []
    for ray in _RAYS:
        end = None
        for ox, oy in ray:
            x, y = x0 + ox, y0 + oy
            if not (0 <= x < W and 0 <= y < H):
                end = (x, y)
                break
            c = y * W + x
            if c in obst or c == other_cell:
                hits.add(c)
                end = (x, y)
                break
            free.add(c)
        ends.append(end or (x0 + ray[-1][0], y0 + ray[-1][1]))
    return hits, free, ends


def clusters(cells):
    """8-connected groups of LiDAR hit cells."""
    left, out = set(cells), []
    while left:
        c = left.pop()
        group, stack = {c}, [c]
        while stack:
            u = stack.pop()
            ux, uy = xy(u)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    v = idx(ux + dx, uy + dy) if 0 <= ux + dx < W and 0 <= uy + dy < H else None
                    if v is not None and v in left:
                        left.discard(v)
                        group.add(v)
                        stack.append(v)
        out.append(group)
    return out


# ------------------------------------------------------------------ random rounds
def random_round(rng, scenario="trap"):
    """scenario "trap"  : chaser starts >= 1.5 m from the goal (earlier assumption)
       scenario "friend": the other team's real setup - chaser starts facing us with a
                          clear line of sight, 1.0-3.0 m away; it does not know our goal"""
    for _ in range(1000):
        obst = set()
        for _b in range(rng.randint(4, 7)):
            w, h = rng.randint(2, 6), rng.randint(2, 6)
            x, y = rng.randint(1, W - w - 1), rng.randint(5, H - h - 6)
            for xx in range(x, x + w):
                for yy in range(y, y + h):
                    obst.add(idx(xx, yy))
        start = idx(rng.randint(2, W - 3), rng.randint(H - 3, H - 1))
        goal = idx(rng.randint(2, W - 3), rng.randint(0, 2))
        if start in obst or goal in obst:
            continue
        d = dijkstra(start, obst)
        if d[goal] >= INF:
            continue
        if scenario == "friend":
            cands = [i for i in range(N) if i not in obst and d[i] < INF
                     and 10 <= cdist(i, start) <= 30 and cdist(i, goal) >= 6
                     and line_of_sight(i, start, obst)]
        else:
            cands = [i for i in range(N) if i not in obst and d[i] < INF
                     and cdist(i, start) >= 12 and cdist(i, goal) >= FAR_FROM_GOAL]
        if not cands:
            continue
        return obst, start, goal, rng.choice(cands)
    raise RuntimeError("could not build a map")


# ================================================================== EVADERS
class Evader:
    name = "evader"
    p = {}                                   # tuning parameters (RAM-Bug fills these)

    def reset(self, goal, capture_cells):
        self.goal = goal
        self.rc = capture_cells
        self.known = set()
        self.d_goal = None
        self.ch_est = None
        self.ch_age = 0
        self.mode = "NAV"
        self.rays = []
        self.lam = 0.5
        self.race_path = []
        self.race_region = set()
        self.seen_free = set()        # cells a LiDAR ray has passed through
        self.free_count = {}          # how many scans saw each cell empty
        self.pending = None           # unconfirmed chaser candidate
        self.robot_cells = set()      # cells where we have seen the other robot (never walls)
        self.ever_moved = False
        self.ch_anchor, self.ch_still = None, 0  # where the track last moved to, scans seen there since
        self.still_cells = set()      # a track sat here for ever: flicker, not motion
        self.anchor = None
        self.detect_how = ""
        self.perc = {"seen": 0, "missed": 0, "false": 0}

    def perceive_chaser(self, hits):
        """LiDAR only: find the other robot among unlabelled hit clusters.
        motion : a hit in a cell we once saw empty  -> something moved there
        shape  : a tiny cluster (1 cell = TurtleBot3 Burger ~0.14 m); boxes are >= 0.2 m
        track  : the cluster closest to where we last saw it (gate 3 cells)"""
        cands = []
        for g in clusters(hits):
            need = self.p.get("motion_min_free", 1)
            moved = [c for c in g if self.free_count.get(c, 0) >= need and c not in self.still_cells]
            gx = [xy(c)[0] for c in g]
            gy = [xy(c)[1] for c in g]
            extent = max(max(gx) - min(gx), max(gy) - min(gy)) + 1
            small = len(g) <= self.p.get("robot_max_cells", 1) and extent <= self.p.get("robot_max_extent", 1)
            if moved and (small or len(moved) == len(g) or len(g) <= 4):
                cands.append((0, moved[0], "motion", g))
            elif small and not any(c in self.still_cells for c in g):
                mx, my = sum(gx) / len(g), sum(gy) / len(g)
                c = min(g, key=lambda q: (xy(q)[0] - mx) ** 2 + (xy(q)[1] - my) ** 2)
                # a small return right next to a known wall is just the visible corner of a box
                touching = any(idx(x + dx, y + dy) in self.known
                               and idx(x + dx, y + dy) not in self.robot_cells
                               and idx(x + dx, y + dy) not in g
                               for (x, y) in zip(gx, gy)
                               for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                               if (dx or dy) and 0 <= x + dx < W and 0 <= y + dy < H)
                if not touching:
                    cands.append((1, c, "shape", g))
        prev, self.prev_motion = getattr(self, "prev_motion", []), [c[1] for c in cands if c[0] == 0]
        if not cands:
            return None, set(), ""
        me = getattr(self, "me", None)
        if self.ch_est is not None:
            near = [c for c in cands if cdist(c[1], self.ch_est) <= 3 + self.ch_age]
            # something moving much closer to us than the one we track is the bigger danger (real
            # robot bag: locked onto wall noise 1.1 m away while the chaser drove into us)
            closer = [] if me is None else [c for c in cands if c[0] == 0 and c not in near and
                                            cdist(c[1], me) + 3 < cdist(self.ch_est, me)]
            if closer:
                cands = closer
            elif near:
                cands = near
        # nothing tracked: a moving blob that was there last scan too (noise jumps about), then the
        # one closest to us. List order picked wall noise while the chaser drove up (real robot bag)
        ref = self.ch_est if self.ch_est is not None else me
        rank, cell, how, group = min(cands, key=lambda c: (
            c[0], self.ch_est is None and not any(cdist(c[1], q) <= 2 for q in prev),
            cdist(c[1], ref) if ref is not None else 0))
        if how == "shape":
            tracking = self.ch_est is not None and self.ch_age == 0 and cdist(cell, self.ch_est) <= 2
            confirmed = self.pending is not None and cdist(cell, self.pending) <= 1.5
            self.pending = cell
            if not (tracking or confirmed):
                return None, set(group), "?"    # new isolated return: wait one scan to confirm
        self.pending = None
        return cell, ({cell} if how == "motion" else group), how

    def sense(self, rnd):
        me = cell_of(rnd.e)
        hits, free, ends = lidar_scan(me, rnd.obst, cell_of(rnd.c))
        if getattr(rnd, "sharing", False):
            # positions are shared over the network: we KNOW where the chaser is
            return self.sense_shared(me, hits, free, ends, cell_of(rnd.c), rnd.c)
        return self.sense_scan(me, hits, free, ends, truth_cell=cell_of(rnd.c))

    def sense_shared(self, me, hits, free, ends, peer_cell, peer_pos):
        """Map from LiDAR, chaser position from the shared (UDP) message."""
        near = {c for c in hits if cdist(c, peer_cell) <= 2}      # its own LiDAR blob is not a wall
        prev = self.ch_est
        self.sense_scan(me, hits - near, free - near, ends)
        self.known -= near
        self.robot_cells |= near
        self.ch_est, self.ch_pos, self.ch_age, self.detect_how = peer_cell, peer_pos, 0, "shared"
        if prev is not None and prev != peer_cell:
            self.ever_moved = True
        return me, prev, peer_cell

    def sense_scan(self, me, hits, free, ends=(), truth_cell=None):
        """Update map + chaser estimate from one LiDAR scan given as grid cells.
        Used by the simulator and by the ROS 2 node (truth_cell only for statistics)."""
        self.rays = list(ends)
        self.me = me
        ch, ch_cells, how = self.perceive_chaser(hits)
        static = hits - ch_cells - self.robot_cells
        if how == "?":
            ch_cells = set()                         # unconfirmed: neither robot nor wall yet
        if ch is not None:
            self.robot_cells.add(ch)
            self.known -= {ch}
        cleared = self.known & free                  # rays passed through: not an obstacle
        new = static - self.known
        if cleared:
            self.known -= cleared
        if new:
            self.known |= new
        self.seen_free |= free
        for c in free:
            self.free_count[c] = self.free_count.get(c, 0) + 1
        if self.d_goal is None or new or cleared:
            self.d_goal = dijkstra(self.goal, self.known)
        # perception statistics (truth is only used for scoring the detector)
        if truth_cell is not None:
            truth = truth_cell in hits
            self.perc["seen"] += truth
            self.perc["missed"] += truth and ch != truth_cell
            self.perc["false"] += ch is not None and ch != truth_cell
        prev = self.ch_est
        if ch is not None:
            if prev is None or self.ch_anchor is None or cdist(ch, prev) > 3 + self.ch_age:
                # a new track, or a jump to another object (outside the tracking gate)
                self.ch_anchor, self.ch_still, self.ever_moved = ch, 0, False
            elif ch != prev:
                self.ever_moved = True
            if cdist(ch, self.ch_anchor) >= 3:
                self.ch_anchor, self.ch_still = ch, 0    # got 0.3 m away: really moving
            else:
                self.ch_still += 1
                if self.ch_still >= self.p.get("forget_still", 8):
                    # seen all the time but stayed put: part of the map, and its cells no longer
                    # count as motion (real robot bag: dodged a box edge for 12 s while the chaser
                    # drove into us). A chaser that only stopped is found again once it moves on
                    x, y = xy(ch)
                    self.still_cells |= {idx(x + dx, y + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                                         if 0 <= x + dx < W and 0 <= y + dy < H}
                    self.robot_cells.discard(ch)
                    self.known.add(ch)
                    self.ch_est, self.ch_age, self.detect_how, self.ever_moved = None, 0, "", False
                    return me, prev, None
            self.ch_est, self.ch_pos, self.ch_age = ch, center(ch), 0
            self.detect_how = how
        elif self.ch_est is not None:
            if not self.ever_moved and self.ch_est in hits:
                # the same spot now looks like part of a wall -> it was a box corner
                self.robot_cells.discard(self.ch_est)
                self.known.add(self.ch_est)
                self.ch_est, self.ch_age, self.detect_how = None, 0, ""
                return me, None, None
            if self.ch_est in free and not self.ever_moved:
                # we can see straight through where it "was" and it never moved:
                # it was a false detection, forget it
                self.robot_cells.discard(self.ch_est)
                self.ch_est, self.ch_age, self.detect_how = None, 0, ""
                return me, None, None
            if not self.ever_moved and self.ch_age >= self.p.get("forget_still", 8):
                # never seen moving and gone for a while: a phantom, or a sleeping trap
                # robot (harmless until we wake it; we will re-detect it when we see it)
                self.robot_cells.discard(self.ch_est)
                self.ch_est, self.ch_age, self.detect_how = None, 0, ""
                return me, None, None
            self.ch_age = min(self.ch_age + 1, 10)   # cap: do not freeze forever
            self.detect_how = "lost"
        return me, prev, ch

    def go_to_goal(self, me):
        best, bd = me, self.d_goal[me]
        for n in neighbours(me, self.known):
            if self.d_goal[n] < bd:
                best, bd = n, self.d_goal[n]
        return best


class AStarReplan(Evader):
    name = "A*-Replan"

    def act(self, rnd):
        me, _, _ = self.sense(rnd)
        self.mode = "NAV"
        return self.go_to_goal(me)


class APF(Evader):
    name = "APF"
    K_REP = 60.0
    RANGE = 10.0

    def act(self, rnd):
        me, _, _ = self.sense(rnd)
        best, bv = me, -INF
        for n in neighbours(me, self.known):
            v = -self.d_goal[n]
            if self.ch_est is not None:
                d = max(0.5, cdist(n, self.ch_est) - self.rc)
                if d < self.RANGE:
                    v -= self.K_REP * (1.0 / d - 1.0 / self.RANGE)
            if v > bv:
                best, bv = n, v
        self.mode = "APF"
        return best


DEFAULT_PARAMS = dict(mu_min=0.5, mu_max=3.0, lam0=0.5, beta=1.5, forget=0.1,
                      w_adv_margin=0.5, w_terr_goal=1.0, w_terr_size=1.0, w_flee=0.3, w_wall=1.0,
                      zone_r=8.0, wake_r=6.0, still_ticks=3, lure_min_dist=10.0, lure_min_dist2=7.0,
                      anchor_min=6, w_orbit_goal=0.3, lure_margin=6.0,
                      w_goal=1.0, w_dist=0.6, w_mob=0.15, dist_cap=10.0, engage=12.0)


class RAMBug(Evader):
    """Race-Aware Adaptive Minimax Bug (our algorithm). See ALGORITHM.md.

      1 SENSE    LiDAR -> known obstacles (unknown = free); chaser if in line of sight
      2 BELIEF   lambda = P(chaser plays rationally | its moves)   (Bayes, online)
      3 RACE     time-race search: only enter cells we reach >= mu ticks before the
                 chaser can get within capture range; mu grows with lambda.
                 goal inside the race region   -> follow that path        (RACE)
                 goal not inside                -> go to the race-safe cell
                                                   closest to the goal     (ADVANCE)
      4 EVADE    no safe cell: pick the move whose time-dominance territory is
                 large and reaches close to the goal, never inside any chaser
                 reply's capture range                                     (EVADE)
    """
    name = "RAM-Bug"

    def __init__(self, params=None, race=True, bayes=True, terr=True, dormant=True, orbit=True, name=None):
        self.use_dormant = dormant
        self.use_orbit = orbit
        self.p = dict(DEFAULT_PARAMS)
        if params:
            self.p.update(params)
        self.use_race, self.use_bayes, self.use_terr = race, bayes, terr
        if name:
            self.name = name

    def reset(self, goal, capture_cells):
        super().reset(goal, capture_cells)
        self.lam = self.p["lam0"]
        self.lam_hist = []
        self.prev_me = None
        self.d_ch = None
        self.still = 0              # ticks the (visible) chaser has not moved
        self.dormant = False
        self.zone_r = self.p["zone_r"]

    # ---------- (2) Bayesian opponent model
    def update_lambda(self, prev_ch, ch_now):
        if not self.use_bayes or prev_ch is None or ch_now is None or self.prev_me is None:
            return
        if ch_now == prev_ch:
            return
        moves = neighbours(prev_ch, self.known, stay=False)
        if ch_now not in moves or len(moves) < 3:
            return
        b = self.p["beta"]

        def soft(target):
            s = [-b * cdist(m, target) for m in moves]
            mx = max(s)
            e = [math.exp(v - mx) for v in s]
            return e[moves.index(ch_now)] / sum(e)

        p_rat = 0.5 * soft(self.prev_me) + 0.5 * soft(self.goal)   # chase us OR head for goal
        p_rnd = 1.0 / len(moves)
        lam = self.lam
        post = lam * p_rat / (lam * p_rat + (1 - lam) * p_rnd)
        post = (1 - self.p["forget"]) * post + self.p["forget"] * self.p["lam0"]
        self.lam = min(0.97, max(0.03, post))

    # ---------- (3) time-race planning
    def race_plan(self, me):
        d_ch = self.d_ch
        mu = self.p["mu_min"] + self.lam * (self.p["mu_max"] - self.p["mu_min"])
        age = 0 if self.dormant else self.ch_age   # a dormant chaser stays put even when hidden
        slack = self.rc + age + 1.0     # +1: chaser position is only known to a cell
        t = {me: 0.0}
        tau = {me: (0.0 if (not self.dormant or self.trips(me)) else INF)}
        parent = {me: None}
        pq = [(0.0, me)]
        region = set()
        found = None
        while pq:
            tu, u = heapq.heappop(pq)
            if tu > t[u]:
                continue
            region.add(u)
            if cdist(u, self.goal) <= GOAL_R:
                found = u
                break
            ux = u % W
            for v in neighbours(u, self.known, stay=False):
                tv = tu + (1.0 if (v % W == ux or v // W == u // W) else SQ2)
                # (5) dormant chaser starts only when we trip its zone
                tau_v = tau[u] if tau[u] < INF else (tv if self.trips(v) else INF)
                if tv + mu < d_ch[v] + tau_v - slack and tv < t.get(v, INF):
                    t[v] = tv
                    tau[v] = tau_v
                    parent[v] = u
                    heapq.heappush(pq, (tv, v))
        self.race_region = region

        def path_to(c):
            path = []
            while c is not None:
                path.append(c)
                c = parent[c]
            return path[::-1]

        if found is not None:
            self.race_path = path_to(found)
            self.mode = "RACE"
            return self.race_path[1] if len(self.race_path) > 1 else me
        if len(region) < 2:
            return None
        k = self.p["w_adv_margin"]

        def score(v):
            return self.d_goal[v] - k * min(d_ch[v] + min(tau[v], 6.0) - slack - t[v], 6.0)

        best = min((v for v in region if v != me), key=score)
        if score(best) >= score(me) - 1e-6:
            return None
        self.race_path = path_to(best)
        self.mode = "ADVANCE"
        return self.race_path[1]

    # ---------- (5) dormant / trap-aware
    def trips(self, v):
        """Would standing on v wake a dormant chaser? (near the goal or near the chaser)"""
        if not self.dormant:
            return True
        return cdist(v, self.goal) <= self.zone_r or cdist(v, self.ch_est) <= self.p["wake_r"]

    def update_dormant(self, prev_ch, ch_now, me):
        if not self.use_dormant or ch_now is None:
            return
        if prev_ch is not None and ch_now == prev_ch:
            self.still += 1
        elif prev_ch is not None:
            if self.dormant:                         # it just woke up: learn the trigger distance
                self.zone_r = max(4.0, min(self.zone_r, cdist(me, self.goal) + 1.0))
            self.still = 0
        self.dormant = self.still >= self.p["still_ticks"]

    def lure(self, me):
        """LURE: trip the trap on purpose at the safest point of the zone edge,
        so the chaser leaves the goal; then normal RAM-Bug takes over."""
        d_me = dijkstra(me, self.known)
        band = [x for x in range(N) if x not in self.known and d_me[x] < INF
                and self.zone_r - 1.0 <= cdist(x, self.goal) <= self.zone_r + 0.5]
        band = [x for x in band if cdist(x, self.ch_est) > self.rc + 4.0]
        edge = [x for x in band if cdist(x, self.ch_est) >= self.p["lure_min_dist"]]
        if self.use_orbit:
            # plan BOTH legs: where to trip the trap AND which pillar to run to afterwards.
            # margin = (chaser time to the hiding spot - capture) - (our time from trip point)
            best = None
            for ring in self.all_rings(d_me):
                for x in band:
                    for r in ring[::2]:
                        m = (self.d_ch[r] - self.rc) - 1.08 * cdist(x, r)
                        if best is None or m - 0.05 * d_me[x] > best[0]:
                            best = (m - 0.05 * d_me[x], m, x, ring)
            if best is not None and best[1] >= self.p["lure_margin"]:
                self.anchor = best[3]
                edge = [best[2]]
            else:
                edge = []                      # no lure we can win -> wait (draw is worth 1)
        if not edge:
            return None
        if cdist(me, self.goal) <= self.zone_r + 1.5:
            # we are on the assumed zone edge and it did not wake up -> the zone is smaller
            self.zone_r = max(2.0, self.zone_r - 0.5)
        # far from the chaser (big margin) but not too far to walk
        if len(edge) == 1:
            tgt = edge[0]
        else:
            tgt = max(edge, key=lambda x: self.d_ch[x] - 0.5 * d_me[x])
        # walk there WITHOUT passing close to the sleeping chaser (it can still catch us)
        keep = self.rc + 4.0
        cx, cy = xy(self.ch_est)
        bubble = {idx(x, y) for x in range(max(0, cx - 7), min(W, cx + 8))
                  for y in range(max(0, cy - 7), min(H, cy + 8)) if cdist(idx(x, y), self.ch_est) <= keep}
        blocked = self.known | (bubble - {me})
        f = dijkstra(tgt, blocked)
        if f[me] >= INF:
            return None
        best, bd = me, f[me]
        for n in neighbours(me, blocked):
            if f[n] < bd:
                best, bd = n, f[n]
        self.race_path = [me, best, tgt]
        self.mode = "LURE"
        return best

    # ---------- (6) anchor obstacle: hide behind it and loop round it
    def anchor_ring(self, me, d_me=None):
        """Pick an obstacle we can run all the way round (not touching the arena edge)
        that is close to us and to the goal. Returns its 'orbit ring' of free cells."""
        if d_me is None:
            d_me = dijkstra(me, self.known)
        best, bscore = None, INF
        for ring in self.all_rings(d_me):
            score = min(d_me[r] for r in ring) + 0.5 * min(self.d_goal[r] for r in ring)
            if score < bscore:
                best, bscore = ring, score
        return best

    def all_rings(self, d_me):
        out = []
        for g in clusters(self.known):
            if len(g) < self.p["anchor_min"]:
                continue
            xs = [xy(c)[0] for c in g]
            ys = [xy(c)[1] for c in g]
            if min(xs) < 3 or min(ys) < 3 or max(xs) > W - 4 or max(ys) > H - 4:
                continue                                 # stuck to the wall: cannot circle it
            near1, near3 = set(), set()
            for c in g:
                n1, n3 = _NEAR13[c]
                near1.update(n1)
                near3.update(n3)
            ring = [r for r in near3 - near1 if r not in self.known and d_me[r] < INF]
            if len(ring) >= 12:
                out.append(ring)
        return out

    def orbit_move(self, rnd, me):
        """ORBIT: keep the obstacle between us and the chaser. Each tick go toward the
        ring cell the chaser needs longest to reach (relative to us), leaning to the goal
        side. When the chaser is on the far side the RACE check opens a path to the goal."""
        d_me = dijkstra(me, self.known)
        ring = self.anchor if self.anchor else self.anchor_ring(me, d_me)
        if not ring:
            return None
        ring = [r for r in ring if r not in self.known and d_me[r] < INF]
        if not ring:
            return None
        k = self.p["w_orbit_goal"]
        tgt = max(ring, key=lambda r: (self.d_ch[r] - self.rc) - d_me[r] - k * self.d_goal[r])
        if (self.d_ch[tgt] - self.rc) - d_me[tgt] < -2:
            return None                                  # chaser is closer to every hiding spot
        if tgt == me:
            return None
        f = dijkstra(tgt, self.known)
        best, bv = None, INF
        for m, p, near in self.safe_moves(rnd, me):
            if near <= self.rc + 0.5:
                continue
            v = f[m] - 0.5 * min(near, 6)
            if v < bv:
                best, bv = m, v
        if best is None:
            return None
        self.race_path = [me, best, tgt]
        self.mode = "ORBIT"
        return best

    # ---------- (4) territory evasion
    def safe_moves(self, rnd, me):
        """my candidate moves with the closest any chaser reply can get (continuous)."""
        out = []
        c_pos = self.ch_pos
        creplies = [advance(c_pos, cn, self.known) for cn in neighbours(self.ch_est, self.known)]
        for m in neighbours(me, self.known):
            p = advance(rnd.e, m, rnd.obst)
            near = min(dist(p, q) for q in creplies) - self.ch_age
            out.append((m, p, near))
        return out

    def territory_move(self, rnd, me):
        d_ch = self.d_ch
        slack = self.rc + self.ch_age + 1.0
        A, B = self.p["w_terr_goal"], self.p["w_terr_size"]
        lim = [dc - slack for dc in d_ch]
        moves = self.safe_moves(rnd, me)
        far = [m for m, p, near in moves if near > self.rc and dist(p, center(self.goal)) > GOAL_R]
        dms = dict(zip(far, dijkstra_many(far, self.known)))      # one call for all candidate moves
        best, bv = me, -INF
        for m, p, near in moves:
            if near <= self.rc:
                v = -1e5 + near
            elif dist(p, center(self.goal)) <= GOAL_R:
                v = 1e5
            else:
                dm = dms[m]
                mine = [g for a, l, g in zip(dm, lim, self.d_goal) if a < l]    # cells we reach first
                size, gbest = len(mine), min(mine, default=INF)
                if size == 0:
                    gbest = self.d_goal[m] + 20
                mob = len(neighbours(m, self.known, stay=False)) / 8.0     # avoid walls / corners
                v = (-A * gbest + B * size / 50.0 + self.p["w_flee"] * min(near, 10)
                     + self.p["w_wall"] * mob)
            if v > bv:
                best, bv = m, v
        return best

    def lookahead(self, rnd, me):
        """AM-Bug: lambda*min + (1-lambda)*mean over chaser replies."""
        p_ = self.p
        best, bv = me, -INF
        c_pos = self.ch_pos
        creplies = [advance(c_pos, cn, self.known) for cn in neighbours(self.ch_est, self.known)]
        for m in neighbours(me, self.known):
            pm = advance(rnd.e, m, rnd.obst)
            mob = len(neighbours(m, self.known, stay=False)) / 8.0
            vals = []
            for q in creplies:
                d = dist(pm, q) - self.ch_age
                if d <= self.rc:
                    vals.append(-1000.0)
                else:
                    vals.append(-p_["w_goal"] * self.d_goal[m] + p_["w_dist"] * min(d - self.rc, p_["dist_cap"])
                                + p_["w_mob"] * mob)
            v = self.lam * min(vals) + (1 - self.lam) * sum(vals) / len(vals)
            if dist(pm, center(self.goal)) <= GOAL_R and min(vals) > -1000.0:
                v += 500.0
            if v > bv:
                best, bv = m, v
        return best

    def act(self, rnd):
        me, prev_ch, ch_now = self.sense(rnd)
        self.update_lambda(prev_ch, ch_now)
        self.lam_hist.append(self.lam)
        self.prev_me = me
        self.race_path, self.race_region = [], set()
        if self.ch_est is None:
            self.mode = "NAV"
            return self.go_to_goal(me)
        self.update_dormant(prev_ch, ch_now, me)
        self.d_ch = dijkstra(self.ch_est, self.known)
        if self.use_race:
            nxt = self.race_plan(me)
            if nxt is not None and not (self.dormant and self.mode == "ADVANCE"):
                return nxt
            if self.dormant:
                lu = self.lure(me)
                if lu is not None:
                    return lu
                if nxt is None:
                    # a sleeping chaser cannot catch us: wait instead of dodging around
                    self.mode = "WAIT"
                    if cdist(me, self.ch_est) > self.p["wake_r"] + 2:
                        return me
            if nxt is not None:
                return nxt
            if self.use_orbit and not self.dormant:
                ob = self.orbit_move(rnd, me)
                if ob is not None:
                    return ob
        elif cdist(me, self.ch_est) > self.p["engage"]:
            self.mode = "NAV"
            return self.go_to_goal(me)
        self.mode = "EVADE"
        return self.territory_move(rnd, me) if self.use_terr else self.lookahead(rnd, me)


# ================================================================== Bug2
DIRS8 = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]   # counter-clockwise

BUG2_PARAMS = dict(avoid_r=8.0,        # cells: a moving object this close -> AVOID
                   avoid_exit=10.0,    # cells: back to Bug2 (new M-line) once it is this far
                   avoid_goal=1.0, avoid_flee=1.5, avoid_mob=0.3,   # AVOID move score weights
                   mline_w=0.75,       # cells: how far a cell centre may be from the M-line
                   goal_view=3.0,      # cells: goal this close and in plain view -> leave the wall
                   max_wall_ticks=80,  # boundary following longer than this -> shortest path (NAV)
                   rush_margin=2.0,    # cells we must stay ahead of the chaser on the way to the goal (RUSH)
                   rush_behind=0.5,    # ... when the chaser is behind us (running away keeps the gap)
                   avoid_ahead=3)      # AVOID: aim this many cells further along the chosen direction


class Bug2(Evader):
    """Bug2 to the goal, plus avoidance of the moving object found by LiDAR (perceive_chaser).

      MLINE : drive along the M-line (straight line from where we started to the goal)
      WALL  : the next M-line cell is blocked -> remember the hit point and follow the obstacle
              boundary (obstacle kept on one side) until we are back on the M-line, closer to the
              goal than the hit point, with the M-line ahead free
      AVOID : the moving object is within avoid_r: take the step that keeps the most distance
              from its worst reply while still heading for the goal; once it is past avoid_exit
              start a new M-line from where we are (the old one is behind us)
      NAV   : went all the way round the obstacle on both sides: shortest path on the map
    The side to go round is the one whose first step is shorter to the goal on the map.
    """
    name = "Bug2"

    def __init__(self, params=None):
        self.p = dict(BUG2_PARAMS)
        if params:
            self.p.update(params)

    def reset(self, goal, capture_cells):
        super().reset(goal, capture_cells)
        self.state = "MLINE"
        self.m_start = None
        self.hit, self.hit_d = None, INF
        self.wall, self.side, self.flipped = None, 1, False
        self.wall_ticks, self.wall_far, self.last_cell = 0, 0.0, None
        self.wall_path, self.rejoins = [], 0  # last tick's boundary-following path, rejoin() in a row
        self.blocked = set()

    # ---------- geometry
    def start_mline(self, me):
        self.state, self.m_start = "MLINE", me
        self.hit, self.hit_d, self.flipped = None, INF, False

    def mline_dist(self, c):
        """Distance (cells) from the centre of c to the M-line segment."""
        ax, ay = xy(self.m_start)
        bx, by = xy(self.goal)
        px, py = xy(c)
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
        return math.hypot(px - ax - t * vx, py - ay - t * vy)

    def free_xy(self, x, y):
        return 0 <= x < W and 0 <= y < H and y * W + x not in self.blocked

    def mline_cands(self, c):
        """Neighbours of c that stay on the M-line and get closer to the goal, best first."""
        x, y = xy(c)
        gd = cdist(c, self.goal)
        out = [idx(x + dx, y + dy) for dx, dy in DIRS8 if 0 <= x + dx < W and 0 <= y + dy < H]
        out = [n for n in out if self.mline_dist(n) <= self.p["mline_w"] and cdist(n, self.goal) < gd - 0.5]
        return sorted(out, key=lambda n: cdist(n, self.goal))

    def mline_next(self, c):
        legal = neighbours(c, self.blocked, stay=False)
        return next((n for n in self.mline_cands(c) if n in legal), None)

    def wall_near(self, c, ref):
        """The blocked neighbour (x, y) of c closest to ref (cells outside the grid count as
        blocked: the arena wall), or None when c is out in the open."""
        x, y = xy(c)
        best = None
        for dx, dy in DIRS8:
            if not self.free_xy(x + dx, y + dy):
                d = (x + dx - ref[0]) ** 2 + (y + dy - ref[1]) ** 2
                if best is None or d < best[0]:
                    best = (d, (x + dx, y + dy))
        return None if best is None else best[1]

    def wall_step(self, c, wall, side):
        """One boundary-following step from c with the blocked cell `wall` on our right (side=1)
        or left (side=-1): turn from the wall toward the open side and take the first free move.
        Returns (next cell, blocked cell next to it to keep following) or (None, wall)."""
        x, y = xy(c)
        k0 = DIRS8.index((wall[0] - x, wall[1] - y))
        last = wall
        for i in range(1, 8):
            dx, dy = DIRS8[(k0 + side * i) % 8]
            if not self.free_xy(x + dx, y + dy):
                last = (x + dx, y + dy)
            elif not (dx and dy) or (self.free_xy(x + dx, y) and self.free_xy(x, y + dy)):
                return idx(x + dx, y + dy), last      # (a diagonal must not cut a corner)
        return None, wall

    def path_ahead(self, me, step, k=4):
        """me + up to k cells of the same rule, for the controller's lookahead."""
        path, c = [me], me
        for _ in range(k):
            c = step(c)
            if c is None or c in path:
                break
            path.append(c)
        return path

    # ---------- modes
    def avoid(self, rnd, me):
        """One-step minimax away from the moving object, still pulled toward the goal."""
        p_ = self.p
        age = min(self.ch_age, 3)
        cap = p_["avoid_exit"] - self.rc
        creplies = [advance(self.ch_pos, cn, self.known) for cn in neighbours(self.ch_est, self.known)]
        best, bv = me, -INF
        for m in neighbours(me, self.blocked):
            p = advance(rnd.e, m, self.blocked)
            near = min(dist(p, q) for q in creplies) - age
            if near <= self.rc:
                v = -1e5 + near
            elif dist(p, center(self.goal)) <= GOAL_R:
                v = 1e5
            else:
                g = self.d_goal[m] if self.d_goal[m] < INF else cdist(m, self.goal) + 50
                mob = len(neighbours(m, self.blocked, stay=False)) / 8.0
                v = -p_["avoid_goal"] * g + p_["avoid_flee"] * min(near - self.rc, cap) + p_["avoid_mob"] * mob
            if v > bv:
                best, bv = m, v
        # aim further along the same direction so the robot runs instead of turning on the spot
        # every half second (the controller only uses the part it can drive in a straight line)
        self.race_path = [me, best]
        (x0, y0), (x1, y1) = xy(me), xy(best)
        dx, dy = x1 - x0, y1 - y0
        for _ in range(p_["avoid_ahead"] if best != me else 0):
            x1, y1 = x1 + dx, y1 + dy
            if not self.free_xy(x1, y1):
                break
            self.race_path.append(idx(x1, y1))
        return best

    def rush_path(self, me):
        """Shortest path to the goal if we reach every cell on it rush_margin cells before the
        chaser can get within capture range of that cell (both at the same speed), else None."""
        if self.d_goal[me] >= INF:
            return None
        first = min(neighbours(me, self.blocked), key=lambda m: self.d_goal[m])
        (x0, y0), (x1, y1), (cx, cy) = xy(me), xy(first), self.ch_pos
        behind = (x1 - x0) * (cx - x0 - 0.5) + (y1 - y0) * (cy - y0 - 0.5) < 0   # chaser behind the first step
        slack = self.rc + self.p["rush_behind" if behind else "rush_margin"] + min(self.ch_age, 3)
        path, c, walked = [me], me, 0.0
        while dist(center(c), center(self.goal)) > GOAL_R:
            n = min(neighbours(c, self.blocked), key=lambda m: self.d_goal[m])
            if self.d_goal[n] >= self.d_goal[c]:
                return None
            walked += cdist(c, n)
            if walked + slack >= dist(self.ch_pos, center(n)):
                return None
            path.append(n)
            c = n
        return path

    def start_wall(self, me):
        cands = self.mline_cands(me)
        ref = xy(cands[0]) if cands else xy(self.goal)
        wall = self.wall_near(me, ref)
        if wall is None:                      # nothing blocked next to us: just restart the line
            self.start_mline(me)
            return False
        self.state, self.hit, self.hit_d, self.wall = "WALL", me, cdist(me, self.goal), wall
        self.wall_ticks, self.wall_far, self.flipped = 0, 0.0, False
        self.wall_path, self.rejoins = [], 0
        opts = []
        for side in (1, -1):
            n, _ = self.wall_step(me, wall, side)
            if n is not None:
                opts.append((self.d_goal[n], -side, side))
        self.side = min(opts)[2] if opts else 1
        return True

    def rejoin(self, me):
        """The furthest cell of last tick's boundary path that we can step to and that still has
        a wall next to it, or None. self.wall is kept: following goes on from there."""
        legal = neighbours(me, self.blocked, stay=False)
        for k in range(len(self.wall_path) - 1, -1, -1):        # its first cell too: a cell border
            c = self.wall_path[k]
            if c in legal and self.wall_near(c, self.wall) is not None:
                self.race_path = [me] + self.wall_path[k:]
                self.wall_path = self.race_path
                return c
        return None

    def follow_wall(self, me):
        """Returns the next cell, or None when Bug2 left the wall (state changed)."""
        gd = cdist(me, self.goal)
        if me != self.hit and self.mline_dist(me) <= self.p["mline_w"] and gd < self.hit_d - 0.5 \
                and self.mline_next(me) is not None:
            self.state = "MLINE"              # Bug2 leave point
            return None
        if gd <= self.p["goal_view"] and line_of_sight(me, self.goal, self.blocked - {me, self.goal}):
            self.start_mline(me)              # the goal is right there in plain view: drive onto it
            self.mode, self.race_path = "MLINE", [me, self.goal]
            return self.goal
        wall = self.wall_near(me, self.wall)
        if wall is not None and self.rejoins < 4 and \
                max(abs(wall[0] - self.wall[0]), abs(wall[1] - self.wall[1])) > 2:
            # only ANOTHER obstacle next to us: the real robot drifted a cell off the boundary while
            # turning. Following that one with the same side drove the wrong way round (real robot
            # bag: target 126 deg behind, ~450 deg of turning). Nothing next to us at all is fine:
            # a new M-line from here (below) was shorter in the simulator. At most 2 s in a row:
            # rejoining on every tick could hover there for ever (one simulator round)
            back = self.rejoin(me)
            if back is not None:
                self.rejoins += 1
                self.wall_ticks += 1
                return back
        if wall is None:                      # the obstacle vanished from the map (it was noise)
            self.start_mline(me)
            return None
        if me != self.last_cell:
            self.wall_ticks += 1
            self.wall_far = max(self.wall_far, cdist(me, self.hit))
        if self.wall_far >= 3 and cdist(me, self.hit) <= 1:
            # back at the hit point without finding a leave point: try the other way round once
            if self.flipped:
                self.state = "NAV"
                return None
            self.side, self.flipped, self.wall_far = -self.side, True, 0.0
        if self.wall_ticks > self.p["max_wall_ticks"]:
            self.state = "NAV"
            return None
        nxt, self.wall = self.wall_step(me, wall, self.side)
        if nxt is None:
            return me
        state = {"wall": self.wall}

        def step(c):                          # same rule from the next cells, without changing state
            n, state["wall"] = self.wall_step(c, state["wall"], self.side)
            return n
        self.race_path = [me] + self.path_ahead(nxt, step, 3)
        self.wall_path, self.rejoins = self.race_path, 0
        return nxt

    def act(self, rnd):
        me, _, _ = self.sense(rnd)
        self.blocked = self.known - {self.goal}     # the goal may sit in the inflated wall ring
        self.race_path, self.race_region = [], set()
        if self.m_start is None:
            self.start_mline(me)
        nxt = self.decide(rnd, me)
        self.last_cell = me
        return nxt

    def decide(self, rnd, me):
        if self.ch_est is not None:
            d = dist(rnd.e, self.ch_pos)
            if d <= self.p["avoid_r"] or (self.state == "AVOID" and d <= self.p["avoid_exit"]):
                self.state = "AVOID"
                path = self.rush_path(me)
                if path is not None:              # we win the race to the goal: just go
                    self.mode = "RUSH"
                    self.race_path = path[:5]
                    return path[1] if len(path) > 1 else me
                self.mode = "AVOID"
                return self.avoid(rnd, me)
        if self.state == "AVOID":                   # it is gone: new M-line from here
            self.start_mline(me)
        for _ in range(3):                          # a mode may hand over to the next one
            if self.state == "NAV":
                self.mode = "NAV"
                nxt = self.go_to_goal(me)
                self.race_path = self.path_ahead(me, lambda c: min(
                    neighbours(c, self.blocked), key=lambda n: self.d_goal[n]))
                return nxt
            if self.state == "MLINE":
                self.mode = "MLINE"
                if self.mline_dist(me) > self.p["mline_w"]:   # pushed off the line (drift): new one
                    self.start_mline(me)
                nxt = self.mline_next(me)
                if nxt is not None:
                    self.race_path = self.path_ahead(me, self.mline_next)
                    return nxt
                if not self.mline_cands(me):        # standing on the goal cell
                    return me
                if not self.start_wall(me):         # hit point
                    continue
            self.mode = "WALL"
            nxt = self.follow_wall(me)
            if nxt is not None:
                return nxt
        return me


# ================================================================== CHASERS
class Chaser:
    name = "chaser"
    knows_goal = False

    def reset(self, rnd):
        self.last_seen = None
        self.rng = random.Random(rnd.seed + 7)
        self.field_cache = {}

    def field(self, rnd, target):
        if target not in self.field_cache:
            if len(self.field_cache) > 200:
                self.field_cache.clear()
            self.field_cache[target] = dijkstra(target, rnd.obst)
        return self.field_cache[target]

    def target_of(self, rnd):
        e = cell_of(rnd.e)
        if rnd.chaser_sight == "omni" or line_of_sight(cell_of(rnd.c), e, rnd.obst):
            self.last_seen = e
        return self.last_seen

    def step_to(self, rnd, f, tie=None):
        me = cell_of(rnd.c)
        best, bd = me, f[me]
        for n in neighbours(me, rnd.obst):
            key = f[n] + (0.01 * cdist(n, tie) if tie is not None else 0) + 1e-6 * self.rng.random()
            if key < bd:
                best, bd = n, key
        return best

    def wander(self, rnd):
        return self.rng.choice(neighbours(cell_of(rnd.c), rnd.obst))


class PursuitChaser(Chaser):
    name = "Pursuit"

    def act(self, rnd):
        t = self.target_of(rnd)
        if t is None:
            return self.wander(rnd)
        if t == cell_of(rnd.c):
            return t
        return self.step_to(rnd, self.field(rnd, t), tie=t)


class PredictChaser(Chaser):
    """Picks the move whose worst case (evader runs away) leaves the smallest distance."""
    name = "Predict"

    def act(self, rnd):
        t = self.target_of(rnd)
        if t is None:
            return self.wander(rnd)
        me = cell_of(rnd.c)
        e_next = [advance(rnd.e, m, rnd.obst) for m in neighbours(t, rnd.obst)]
        f = self.field(rnd, t)
        best, bv = me, INF
        for n in neighbours(me, rnd.obst):
            q = advance(rnd.c, n, rnd.obst)
            worst = max(dist(q, p) for p in e_next)
            v = worst + 0.5 * f[n] + 1e-6 * self.rng.random()
            if v < bv:
                best, bv = n, v
        return best


class InterceptChaser(Chaser):
    """Infers the evader's goal with Bayes over candidate cells, then cuts it off."""
    name = "Intercept"
    BETA = 2.0

    def reset(self, rnd):
        super().reset(rnd)
        if self.knows_goal:
            self.cands = [rnd.goal]
        else:
            self.cands = [idx(x, y) for x in range(2, W, 5) for y in range(2, H, 5)
                          if idx(x, y) not in rnd.obst]
        self.fields = [dijkstra(g, rnd.obst) for g in self.cands]
        self.belief = [1.0 / len(self.cands)] * len(self.cands)
        self.prev_e = None

    def act(self, rnd):
        e = self.target_of(rnd)
        if e is None:
            return self.wander(rnd)
        if self.prev_e is not None and e != self.prev_e and len(self.cands) > 1:
            post = [b * math.exp(self.BETA * (f[self.prev_e] - f[e])) for b, f in zip(self.belief, self.fields)]
            s = sum(post)
            self.belief = [0.97 * p / s + 0.03 / len(post) for p in post]
        self.prev_e = e
        g = max(range(len(self.cands)), key=lambda k: self.belief[k])
        f = self.fields[g]
        d_me = self.field(rnd, cell_of(rnd.c))
        rc = rnd.capture_cells
        cur, t, target = e, 0.0, None
        for _ in range(80):
            if d_me[cur] - rc <= t:
                target = cur
                break
            nxt = min(neighbours(cur, rnd.obst, stay=False), key=lambda n: f[n], default=cur)
            if f[nxt] >= f[cur]:
                break
            t += cdist(cur, nxt)
            cur = nxt
        if target is None:
            target = cur if self.knows_goal else e
        if target == cell_of(rnd.c):
            target = e
        return self.step_to(rnd, self.field(rnd, target), tie=e)


class GuardChaser(InterceptChaser):
    name = "Guard"
    knows_goal = True


class TrapChaser(Chaser):
    """The other team's described strategy: scatter 'particles' around the goal
    (a trip-wire zone). Wait until our robot passes over a particle, then run at us.
      wait="start" : waits where it starts        (Trap)
      wait="goal"  : first parks next to the goal  (Trap-Goal)
    """
    name = "Trap"
    WAIT = "start"
    ZONE_R = 6.0          # particles within 0.6 m of the goal
    N_PART = 40
    DETECT = 1.5          # we trip a particle when within 0.15 m of it

    def reset(self, rnd):
        super().reset(rnd)
        gx, gy = center(rnd.goal)
        self.particles = []
        tries = 0
        while len(self.particles) < self.N_PART and tries < 5000:
            tries += 1
            a = self.rng.uniform(0, 2 * math.pi)
            r = self.ZONE_R * math.sqrt(self.rng.random())
            p = (gx + r * math.cos(a), gy + r * math.sin(a))
            if 0 <= p[0] < W and 0 <= p[1] < H and cell_of(p) not in rnd.obst:
                self.particles.append(p)
        self.triggered = False
        self.park = None
        if self.WAIT == "goal":
            # park 0.4 m from the goal on the side facing the evader's start
            d = self.field(rnd, rnd.goal)
            ring = [i for i in range(N) if i not in rnd.obst and 3.5 <= cdist(i, rnd.goal) <= 4.5]
            if ring:
                self.park = min(ring, key=lambda i: cdist(i, rnd.start) + 0.3 * d[i])

    def act(self, rnd):
        if not self.triggered and any(dist(rnd.e, p) <= self.DETECT for p in self.particles):
            self.triggered = True
        if self.triggered:
            e = cell_of(rnd.e)
            return self.step_to(rnd, self.field(rnd, e), tie=e)
        me = cell_of(rnd.c)
        if self.park is not None and me != self.park:
            return self.step_to(rnd, self.field(rnd, self.park))
        return me


class TrapGoalChaser(TrapChaser):
    name = "Trap-Goal"
    WAIT = "goal"


class FriendChaser(Chaser):
    """The other team's chaser (their code of Oct 2026, as described in their chat):
    - knows our position and velocity (positions are shared over UDP), not our goal
    - farther than 0.8 m: INTERCEPT, aim at p_evader + v_evader * t_intercept
    - closer than 0.8 m: CHASE straight at us (small lead); back to intercept above 1.0 m
    - our velocity is smoothed; below 0.10 m/s it aims at us directly
    """
    name = "Friend"
    CHASE_IN, CHASE_OUT = 8.0, 10.0      # cells (0.8 m / 1.0 m)

    def reset(self, rnd):
        super().reset(rnd)
        self.prev = None
        self.v = (0.0, 0.0)
        self.close = False

    def act(self, rnd):
        e = rnd.e
        if self.prev is not None:
            vx, vy = e[0] - self.prev[0], e[1] - self.prev[1]     # cells per tick
            self.v = (0.6 * self.v[0] + 0.4 * vx, 0.6 * self.v[1] + 0.4 * vy)
        self.prev = e
        d = dist(e, rnd.c)
        if d < self.CHASE_IN:
            self.close = True
        elif d > self.CHASE_OUT:
            self.close = False
        vx, vy = self.v
        if math.hypot(vx, vy) < 0.5:          # < 0.1 m/s: treat as standing still
            vx = vy = 0.0
        t = 1.0 if self.close else d / STEP    # ticks to reach us
        tx = min(W - 0.5, max(0.5, e[0] + vx * t))
        ty = min(H - 0.5, max(0.5, e[1] + vy * t))
        tgt = cell_of((tx, ty))
        if tgt in rnd.obst:
            tgt = cell_of(e)
        self.mode = "CHASE" if self.close else "INTERCEPT"
        return self.step_to(rnd, self.field(rnd, tgt), tie=cell_of(e))


class RandomChaser(Chaser):
    name = "Random"

    def reset(self, rnd):
        super().reset(rnd)
        self.tgt = None

    def act(self, rnd):
        me = cell_of(rnd.c)
        if self.tgt is None or self.tgt == me or self.rng.random() < 0.05:
            self.tgt = self.rng.choice([i for i in range(N) if i not in rnd.obst])
        return self.step_to(rnd, self.field(rnd, self.tgt))


class HumanChaser(Chaser):
    name = "Human"

    def __init__(self):
        self.dir = (0, 0)

    def act(self, rnd):
        me = cell_of(rnd.c)
        if self.dir == (0, 0):
            return me
        x, y = xy(me)
        opts = neighbours(me, rnd.obst)
        for dx, dy in (self.dir, (self.dir[0], 0), (0, self.dir[1])):
            nx, ny = x + dx, y + dy
            if (dx or dy) and 0 <= nx < W and 0 <= ny < H and idx(nx, ny) in opts:
                return idx(nx, ny)
        return me


EVADERS = ["RAM-Bug", "AM-Bug", "A*-Replan", "APF"]
ABLATION = ["RAM-Bug", "RAM-noOrbit", "RAM-noRace", "RAM-noTerr", "RAM-noBayes", "RAM-noDormant"]
CHASERS = ["Friend", "Pursuit", "Intercept", "Predict", "Guard", "Trap", "Trap-Goal", "Random", "Human"]


def load_params():
    """Tuned RAM-Bug parameters from tune.py (best_params.json), if present."""
    import json
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best_params.json")
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            return json.load(f)["params"]
    return None


def make_evader(kind, params=None):
    if params is None and kind not in ("A*-Replan", "APF", "Bug2"):
        params = load_params()
    if kind == "RAM-Bug":
        return RAMBug(params)
    if kind == "AM-Bug":
        return RAMBug(params, race=False, terr=False, name="AM-Bug")
    if kind == "RAM-noRace":
        return RAMBug(params, race=False, name="RAM-noRace")
    if kind == "RAM-noTerr":
        return RAMBug(params, terr=False, name="RAM-noTerr")
    if kind == "RAM-noOrbit":
        return RAMBug(params, orbit=False, name="RAM-noOrbit")
    if kind == "RAM-noDormant":
        return RAMBug(params, dormant=False, name="RAM-noDormant")
    if kind == "RAM-noBayes":
        return RAMBug(params, bayes=False, name="RAM-noBayes")
    if kind == "Bug2":
        return Bug2()
    if kind == "A*-Replan":
        return AStarReplan()
    if kind == "APF":
        return APF()
    raise ValueError(kind)


def make_chaser(kind):
    return {"Friend": FriendChaser, "Pursuit": PursuitChaser, "Intercept": InterceptChaser, "Predict": PredictChaser,
            "Guard": GuardChaser, "Trap": TrapChaser, "Trap-Goal": TrapGoalChaser, "Random": RandomChaser, "Human": HumanChaser}[kind]()


# ================================================================== round
class Round:
    def __init__(self, evader, chaser, seed=0, capture_r=0.3, T=150, chaser_sight="omni",
                 layout=None, sharing=None, scenario=None):
        """sharing: evader gets the chaser position from the network (default: on for Friend)
           scenario: "friend" (chaser starts in view, 1-3 m away) or "trap" (default: by chaser)"""
        self.seed = seed
        friend = getattr(chaser, "name", "") == "Friend"
        self.sharing = friend if sharing is None else sharing
        scenario = scenario or ("friend" if friend else "trap")
        self.rng = random.Random(seed)
        self.capture_r = capture_r
        self.capture_cells = capture_r / CELL
        self.T = T
        self.chaser_sight = chaser_sight
        if layout is None:
            layout = random_round(self.rng, scenario)
        self.obst, self.start, self.goal, self.c_start = layout
        self.evader, self.chaser = evader, chaser
        self.reset()

    def layout(self):
        return (self.obst, self.start, self.goal, self.c_start)

    def reset(self):
        self.e, self.c = center(self.start), center(self.c_start)
        self.tick = 0
        self.outcome = None
        self.trail_e, self.trail_c = [self.e], [self.c]
        self.min_dist = dist(self.e, self.c) * CELL
        self.evader.reset(self.goal, self.capture_cells)
        self.chaser.reset(self)

    def step(self):
        if self.outcome:
            return self.outcome
        te = self.evader.act(self)
        tc = self.chaser.act(self)
        self.e = advance(self.e, te, self.obst)
        self.c = advance(self.c, tc, self.obst)
        self.tick += 1
        self.trail_e.append(self.e)
        self.trail_c.append(self.c)
        d = dist(self.e, self.c)
        self.min_dist = min(self.min_dist, d * CELL)
        if d <= self.capture_cells:
            self.outcome = "caught"
        elif dist(self.e, center(self.goal)) <= GOAL_R:
            self.outcome = "reached"
        elif self.tick >= self.T:
            self.outcome = "timeout"
        return self.outcome

    def play(self):
        while not self.outcome:
            self.step()
        return self.outcome
