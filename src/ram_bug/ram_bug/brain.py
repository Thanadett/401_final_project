"""
RAM-Bug brain - everything the robot thinks, with NO ROS imports
================================================================
The ROS node (ram_bug_node.py) only moves data in and out of this class, so the
same code can be tested offline (test/test_brain_offline.py).

Frames
  arena : origin at a CORNER of the 3 x 3 m arena, x along one side, y along the other.
          The goal is given in this frame. Grid cell (i, j) covers
          [i*0.1, (i+1)*0.1) x [j*0.1, (j+1)*0.1).
  odom  : TurtleBot3 odometry, starts at (0, 0, 0) where the robot was switched on.
          arena = start_pose (+) odom   (start_x, start_y, start_yaw parameters)

Pipeline
  plan()     2 Hz  : LaserScan -> hit / free grid cells -> planner decision (target cell)
                     planner = Bug2 (default, cfg algorithm: bug2) or RAM-Bug (algorithm: ram_bug)
  control()  20 Hz : drive toward the target with a heading controller -> (v, w)
"""
import math

from . import eg_core as eg

CELL = eg.CELL
ARENA = eg.W * CELL
# LiDAR hits on the arena walls land just outside the grid, so the walls never enter the map.
# The outer ring of cells (centres 0.05 m from the wall) is closer than the Burger's half width.
# the ray-marching steps t = 0.05, 0.10, ... built by the same repeated addition as the loop
_STEPS = [0.05]
while _STEPS[-1] < 4.0:
    _STEPS.append(_STEPS[-1] + 0.05)
_STEPS = eg._np.array(_STEPS) if eg._np is not None else None
BORDER = frozenset(eg.idx(i, j) for i in range(eg.W) for j in range(eg.H)
                   if i in (0, eg.W - 1) or j in (0, eg.H - 1))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RosPlanner:
    """Planner (RAM-Bug or Bug2) fed by a real LaserScan instead of the simulator.
    Keeps the raw obstacle cells separate and plans on an inflated copy so the
    0.14 m wide Burger does not scrape boxes. Mixed in before the eg_core planner class."""

    def __init__(self, params, inflate=1, detect="lidar", forget_ticks=12, motion_confirm=True,
                 shape_detect=False):
        super().__init__(params)
        self.inflate = inflate
        self._scan = (set(), set())
        self.detect = detect                  # "lidar" or "off" (no LiDAR chaser detection)
        self.forget_ticks = forget_ticks      # drop a LiDAR estimate not seen for this many plans
        self.motion_confirm = motion_confirm
        self.pending_motion = None
        self.shape_detect = shape_detect      # also accept a small STILL blob as the chaser
        self.turning = False                  # set by Brain: no chaser detection while turning fast

    def reset(self, goal, capture_cells):
        super().reset(goal, capture_cells)
        self.raw = set()

    def set_scan(self, hits, free):
        self._scan = (hits, free)

    def perceive_chaser(self, hits):
        """Stricter version for the real robot (LiDAR noise, odometry lag):
        - ignore returns on the arena border (walls, people standing outside)
        - a "motion" detection must not touch a known obstacle (a box edge that jitters
          between two cells looks like motion) and must be seen in 2 scans in a row."""
        if self.detect == "off" or (self.turning and self.ch_est is None):
            return None, set(), ""
        inner = {c for c in hits if 0 < eg.xy(c)[0] < eg.W - 1 and 0 < eg.xy(c)[1] < eg.H - 1}
        cell, group, how = super().perceive_chaser(inner)
        if how in ("shape", "?") and not self.shape_detect and not (
                self.ch_est is not None and cell is not None and eg.cdist(cell, self.ch_est) <= 2):
            # a small still blob is usually a box corner / chair leg: treat it as a wall
            return None, set(), ""
        if how != "motion" or not self.motion_confirm:
            if how != "motion":
                self.pending_motion = None
            return cell, group, how
        if self.ch_est is not None and self.ch_age == 0 and eg.cdist(cell, self.ch_est) <= 2:
            return cell, group, how                        # already tracking it
        x, y = eg.xy(cell)
        # Only a real box counts here. A stored "wall" cell that LiDAR saw empty before is
        # something that moved in - usually the chaser itself, mapped while it stood still,
        # whose old cells would otherwise veto every detection as it drives off.
        need = self.p.get("motion_min_free", 1)
        touching = any(n in self.known and n not in self.robot_cells and self.free_count.get(n, 0) < need
                       for n in (eg.idx(x + dx, y + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                                 if (dx or dy) and 0 <= x + dx < eg.W and 0 <= y + dy < eg.H))
        confirmed = self.pending_motion is not None and eg.cdist(cell, self.pending_motion) <= 2
        self.pending_motion = cell
        if touching or not confirmed:
            return None, set(group), "?"
        self.pending_motion = None
        return cell, group, how

    def _inflated(self, me):
        if self.inflate <= 0:
            return set(self.raw)
        out = set()
        k = self.inflate
        for c in self.raw:
            x, y = eg.xy(c)
            for dx in range(-k, k + 1):
                for dy in range(-k, k + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < eg.W and 0 <= ny < eg.H:
                        out.add(eg.idx(nx, ny))
        out |= BORDER
        # never block our own cell; if we are boxed in by the inflation (after drift),
        # also free the cells around us so the robot can drive out
        out.discard(me)
        if not eg.neighbours(me, out, stay=False):
            mx, my = eg.xy(me)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if 0 <= mx + dx < eg.W and 0 <= my + dy < eg.H:
                        out.discard(eg.idx(mx + dx, my + dy))
        return out

    def sense(self, view):
        me = eg.cell_of(view.e)
        hits, free = self._scan
        self.known = self.raw                    # perception works on the raw map
        peer = getattr(view, "peer", None)       # (x, y) in cells from position sharing
        if peer is not None:
            out = self.sense_shared(me, hits, free, (), eg.cell_of(peer), peer)
        else:
            out = self.sense_scan(me, hits, free)
            if self.ch_est is not None and self.ch_age >= self.forget_ticks:
                # not seen for a while: probably a false detection -> forget it
                self.robot_cells.discard(self.ch_est)
                self.ch_est, self.ch_age, self.detect_how, self.ever_moved = None, 0, "", False
        self.raw = self.known
        self.known = self._inflated(me)          # planning uses the inflated map
        self.d_goal = eg.dijkstra(self.goal, self.known)
        view.obst = self.known
        return out


class RosRAMBug(RosPlanner, eg.RAMBug):
    pass


class RosBug2(RosPlanner, eg.Bug2):
    pass


class View:
    """What eg_core expects from the 'world': our position (in cells) and obstacles."""

    def __init__(self, e, obst, peer=None):
        self.e = e
        self.obst = obst
        self.c = None
        self.peer = peer


class Brain:
    def __init__(self, cfg, tuned_params=None):
        """cfg: dict of node parameters (see config/ram_bug.yaml)."""
        self.cfg = cfg
        p = dict(tuned_params or {})
        p.update(dict(robot_max_cells=cfg.get("robot_max_cells", 4),
                      robot_max_extent=cfg.get("robot_max_extent", 3),
                      motion_min_free=cfg.get("motion_min_free", 2)))
        # Bug2: how close (m) a moving object may come before we step away, and when it is gone
        p.update(dict(avoid_r=cfg.get("avoid_r", 1.2) / CELL, avoid_exit=cfg.get("avoid_exit", 1.4) / CELL))
        self.params = p
        self.algorithm = str(cfg.get("algorithm", "bug2")).lower().replace("-", "_")
        if self.algorithm not in ("bug2", "ram_bug"):
            raise ValueError(f"algorithm '{self.algorithm}' unknown: use bug2 or ram_bug")
        # plan with a bigger capture radius: the real robot must turn before it can flee
        self.capture_cells = (cfg.get("capture_r", 0.30) + cfg.get("plan_margin", 0.0)) / CELL
        self.goal = None
        self.planner = None
        self.target = None          # (x, y) in arena metres
        self.mode = "IDLE"
        self.done = None            # "reached" / "timeout"
        self.t_start = None
        self.last_hits = set()
        self.peer = None            # (x, y, time) of the chaser from position sharing (arena metres)
        self.peer_used = False
        self.last_mode = None
        self.last_v = 0.0
        self.stuck_ref = None
        self.stuck_since = None
        self.recover_until = None
        self.recover_w = 1.2
        self.blocked = False
        self.start_xy = None        # where the robot was when it got the goal
        self.has_moved = False

    # ------------------------------------------------------------ frames
    def odom_to_arena(self, ox, oy, oyaw):
        sx, sy, syaw = self.cfg["start_x"], self.cfg["start_y"], self.cfg["start_yaw"]
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + c * ox - s * oy, sy + s * ox + c * oy, wrap(syaw + oyaw)

    def arena_to_odom(self, ax, ay):
        sx, sy, syaw = self.cfg["start_x"], self.cfg["start_y"], self.cfg["start_yaw"]
        c, s = math.cos(-syaw), math.sin(-syaw)
        dx, dy = ax - sx, ay - sy
        return c * dx - s * dy, s * dx + c * dy

    @staticmethod
    def cell_of_xy(x, y):
        i, j = int(math.floor(x / CELL)), int(math.floor(y / CELL))
        if 0 <= i < eg.W and 0 <= j < eg.H:
            return eg.idx(i, j)
        return None

    # ------------------------------------------------------------ position sharing
    def set_peer(self, x, y, now):
        """Chaser position received from the other robot (arena frame, metres)."""
        self.peer = (x, y, now)

    # ------------------------------------------------------------ goal
    def set_goal(self, gx, gy):
        cell = self.cell_of_xy(gx, gy)
        if cell is None:
            raise ValueError(f"goal ({gx:.2f}, {gy:.2f}) is outside the 3 x 3 m arena")
        self.goal = (gx, gy)
        cls = RosBug2 if self.algorithm == "bug2" else RosRAMBug
        self.planner = cls(self.params, inflate=self.cfg.get("inflate_cells", 1),
                           detect=str(self.cfg.get("detect_chaser", "lidar")).lower(),
                           forget_ticks=int(self.cfg.get("forget_ticks", 12)),
                           motion_confirm=bool(self.cfg.get("motion_confirm", True)),
                           shape_detect=bool(self.cfg.get("shape_detect", False)))
        self.planner.reset(cell, self.capture_cells)
        self.done = None
        self.t_start = None
        self.mode = "START"
        self.start_xy, self.has_moved = None, False

    # ------------------------------------------------------------ LiDAR -> cells
    def scan_to_cells(self, x, y, yaw, ranges, angle_min, angle_inc, range_min, range_max):
        off = self.cfg.get("laser_x_offset", -0.032)
        lx, ly = x + off * math.cos(yaw), y + off * math.sin(yaw)
        max_free = min(range_max, self.cfg.get("max_free_range", 3.0))
        hits, free = set(), set()
        floor, w, h = math.floor, eg.W, eg.H
        rays = []                                          # (cos, sin, stop) of every ray that frees cells
        for i, r in enumerate(ranges):
            a = yaw + angle_min + i * angle_inc
            ca, sa = math.cos(a), math.sin(a)
            valid = (r is not None) and math.isfinite(r) and range_min < r < range_max
            if not valid and not (r is not None and math.isinf(r) and r > 0):
                continue                                   # 0 / NaN = no reading
            reach = r if valid else max_free
            if valid:
                c = self.cell_of_xy(lx + r * ca, ly + r * sa)
                if c is not None:
                    hits.add(c)
            rays.append((ca, sa, min(reach, max_free) - 0.15))   # keep clear of the hit cell (noise)
        if eg._np is not None and rays:
            free = self._free_cells_np(lx, ly, rays)
        else:
            for ca, sa, stop in rays:
                t = 0.05
                while t < stop:                            # cell_of_xy inlined: the hot loop on a Pi
                    ci, cj = floor((lx + t * ca) / CELL), floor((ly + t * sa) / CELL)
                    if not (0 <= ci < w and 0 <= cj < h):
                        break
                    free.add(cj * w + ci)
                    t += 0.05
        free -= hits
        return hits, free

    @staticmethod
    def _free_cells_np(lx, ly, rays):
        """Same cells as the loop above, every ray at once: a ray frees the cells at 0.05 m steps
        up to its stop, but nothing after it first leaves the grid."""
        np = eg._np
        ca, sa, stop = (np.array(col) for col in zip(*rays))
        t = _STEPS[None, :]
        ci = np.floor((lx + t * ca[:, None]) / CELL)
        cj = np.floor((ly + t * sa[:, None]) / CELL)
        inside = np.logical_and.accumulate((ci >= 0) & (ci < eg.W) & (cj >= 0) & (cj < eg.H), axis=1)
        take = inside & (t < stop[:, None])
        return set((cj[take] * eg.W + ci[take]).astype(np.int64).tolist())

    def clear_line(self, x, y, cx, cy, blocked, me):
        """True if the segment from (x, y) m to (cx, cy) cells crosses no blocked cell."""
        tx, ty = cx * CELL, cy * CELL
        n = max(1, int(math.hypot(tx - x, ty - y) / 0.02))
        for i in range(1, n + 1):
            c = self.cell_of_xy(x + (tx - x) * i / n, y + (ty - y) * i / n)
            if c is None or (c in blocked and c != me):
                return False
        return True

    # ------------------------------------------------------------ 2 Hz decision
    def plan(self, x, y, yaw, scan, now, scan_pose=None):
        """scan = (ranges, angle_min, angle_inc, range_min, range_max).
        scan_pose = (x, y, yaw, w) of the robot when that scan arrived (arena frame, w = turn
        rate). The scan is projected from that pose; while the robot turns fast the scan is
        smeared, so it is not used for the map / chaser detection at all."""
        if self.planner is None or self.done:
            return
        if self.t_start is None:
            self.t_start = now
        if math.hypot(x - self.goal[0], y - self.goal[1]) <= eg.GOAL_R * CELL:
            self.done, self.mode, self.target = "reached", "DONE", None
            return
        if now - self.t_start >= self.cfg.get("time_limit", 75.0):
            self.done, self.mode, self.target = "timeout", "TIME UP", None
            return
        me = self.cell_of_xy(x, y)
        if me is None:
            self.mode, self.target = "OUTSIDE", self.goal
            return
        # turning fast smears the scan: still map with it, but do not look for a NEW chaser
        self.planner.turning = scan_pose is not None and abs(scan_pose[3]) > self.cfg.get("turn_gate", 0.6)
        sx_, sy_, syaw_ = scan_pose[:3] if scan_pose is not None else (x, y, yaw)
        hits, free = self.scan_to_cells(sx_, sy_, syaw_, *scan)
        self.last_hits = hits
        self.planner.set_scan(hits, free)
        peer = None
        if self.peer is not None and now - self.peer[2] <= self.cfg.get("peer_timeout", 0.6):
            px, py = self.peer[0] / CELL, self.peer[1] / CELL
            if 0 <= px < eg.W and 0 <= py < eg.H:
                peer = (px, py)
        self.peer_used = peer is not None
        view = View((x / CELL, y / CELL), self.planner.known, peer)
        nxt = self.planner.act(view)
        self.mode = self.planner.mode
        path = self.planner.race_path or []
        near_goal = math.hypot(x - self.goal[0], y - self.goal[1]) < 0.35
        if self.mode == "RACE" and (len(path) < 2 or near_goal) or self.mode in ("NAV", "MLINE") and near_goal:
            # last few cm: drive straight onto the goal (it may lie in the blocked wall ring)
            self.target = self.goal
            self.last_mode = self.mode
            return
        if nxt == me and self.mode != "WAIT" and self.planner.d_goal[me] >= eg.INF:
            # no path to the goal in our map (a wall line across the arena: usually a wrong
            # start_x/start_y/start_yaw, or the goal is inside a box). Try without inflation,
            # otherwise drive straight at the goal (stop_dist still prevents crashes).
            raw = getattr(self.planner, "raw", set())
            d = eg.dijkstra(self.planner.goal, raw - {me})
            better = [c for c in eg.neighbours(me, raw) if d[c] < d[me]]
            if better:
                nxt = min(better, key=lambda c: d[c])
                self.mode = "NO PATH (thin)"
            else:
                self.mode, self.target = "NO PATH -> straight", self.goal
                self.last_mode = self.mode
                return
        if self.mode == "WAIT" or nxt == me and len(path) < 2:
            self.target = None
            return
        # aim a little ahead along the planned path for smoother driving
        # but only as far as the straight line stays clear of the inflated map: the robot
        # drives that line, and cutting a box corner scrapes the box with the wheels
        aim = nxt
        look = self.cfg.get("lookahead_cells", 3)
        blocked = self.planner.known
        if len(path) >= 3 and path[0] == me:
            for k in range(min(look, len(path) - 1), 0, -1):
                if self.clear_line(x, y, *eg.center(path[k]), blocked, me):
                    aim = path[k]
                    break
        cx, cy = eg.center(aim)
        new = (cx * CELL, cy * CELL)
        # never aim too close: a target one cell away makes the heading error jump when we
        # are off the cell centre, and we stop on it until the next plan (0.5 s later)
        d_aim = math.hypot(new[0] - x, new[1] - y)
        min_aim = self.cfg.get("min_aim", 0.25)
        if 1e-6 < d_aim < min_aim:
            k = min_aim / d_aim
            far = (min(ARENA - 0.05, max(0.05, x + k * (new[0] - x))),
                   min(ARENA - 0.05, max(0.05, y + k * (new[1] - y))))
            if self.clear_line(x, y, far[0] / CELL, far[1] / CELL, blocked, me):
                new = far
        # smooth only tiny changes of direction (the grid planner can flip between two
        # equally good cells every tick). Real corners (45 deg) are taken at once.
        if self.target is not None and self.mode == self.last_mode:
            a_old = math.atan2(self.target[1] - y, self.target[0] - x)
            a_new = math.atan2(new[1] - y, new[0] - x)
            if abs(wrap(a_new - a_old)) < math.radians(self.cfg.get("smooth_deg", 15.0)):
                new = (0.5 * (new[0] + self.target[0]), 0.5 * (new[1] + self.target[1]))
        self.last_mode = self.mode
        self.target = new

    # ------------------------------------------------------------ 20 Hz control
    def control(self, x, y, yaw, front_min, now=None):
        """Returns (v, w). front_min = closest LiDAR range in +-30 deg in front."""
        if self.target is None:
            self.stuck_ref, self.stuck_since = None, None
            return 0.0, 0.0
        tx, ty = self.target
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        # stuck recovery: we keep asking to drive but do not move -> back up and turn.
        # "asking to drive" includes being held by the front safety stop: otherwise a target
        # straight behind a box corner leaves v = 0 and w = 0 for ever
        # The no-progress timer only runs once the robot has moved at all: before that the first
        # commands may simply not have reached it yet (DDS discovery, bringup still starting)
        if self.start_xy is None:
            self.start_xy = (x, y)
        if not self.has_moved and math.hypot(x - self.start_xy[0], y - self.start_xy[1]) > 0.03:
            self.has_moved = True
        if now is not None:
            if self.recover_until and now < self.recover_until:
                return -0.08, self.recover_w
            if self.last_v > 0.05 and (self.has_moved or self.blocked):
                limit = self.cfg.get("blocked_time", 0.5) if self.blocked else self.cfg.get("stuck_time", 2.0)
                if self.stuck_ref is None or math.hypot(x - self.stuck_ref[0], y - self.stuck_ref[1]) > 0.03:
                    self.stuck_ref, self.stuck_since = (x, y), now
                elif self.stuck_since is not None and now - self.stuck_since > limit:
                    self.recover_until = now + 0.6
                    self.recover_w = 1.2 if err >= 0 else -1.2     # turn toward the target side
                    self.stuck_ref, self.stuck_since = None, None
                    return -0.08, self.recover_w
        vmax = self.cfg.get("max_lin", 0.20)
        wmax = self.cfg.get("max_ang", 2.5)
        d = math.hypot(tx - x, ty - y)
        if d < 0.02:
            return 0.0, 0.0
        w = max(-wmax, min(wmax, self.cfg.get("k_ang", 3.0) * err))
        if abs(err) > math.radians(75):
            v = 0.0                                        # turn on the spot first
        else:
            v = vmax * max(0.0, math.cos(err))
        self.last_v = v
        self.blocked = front_min < self.cfg.get("stop_dist", 0.17)
        if self.blocked:
            v = 0.0                                        # safety: something right in front
        return v, w
