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
  plan()     2 Hz  : LaserScan -> hit / free grid cells -> RAM-Bug decision (target cell)
  control()  20 Hz : drive toward the target with a heading controller -> (v, w)
"""
import math

from . import eg_core as eg

CELL = eg.CELL


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RosRAMBug(eg.RAMBug):
    """RAM-Bug fed by a real LaserScan instead of the simulator.
    Keeps the raw obstacle cells separate and plans on an inflated copy so the
    0.14 m wide Burger does not scrape boxes."""

    def __init__(self, params, inflate=1):
        super().__init__(params)
        self.inflate = inflate
        self._scan = (set(), set())

    def reset(self, goal, capture_cells):
        super().reset(goal, capture_cells)
        self.raw = set()

    def set_scan(self, hits, free):
        self._scan = (hits, free)

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
        self.raw = self.known
        self.known = self._inflated(me)          # planning uses the inflated map
        self.d_goal = eg.dijkstra(self.goal, self.known)
        view.obst = self.known
        return out


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
        self.params = p
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
        self.planner = RosRAMBug(self.params, inflate=self.cfg.get("inflate_cells", 1))
        self.planner.reset(cell, self.capture_cells)
        self.done = None
        self.t_start = None
        self.mode = "START"

    # ------------------------------------------------------------ LiDAR -> cells
    def scan_to_cells(self, x, y, yaw, ranges, angle_min, angle_inc, range_min, range_max):
        off = self.cfg.get("laser_x_offset", -0.032)
        lx, ly = x + off * math.cos(yaw), y + off * math.sin(yaw)
        max_free = min(range_max, self.cfg.get("max_free_range", 3.0))
        hits, free = set(), set()
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
            t = 0.05
            stop = min(reach, max_free) - 0.15             # keep clear of the hit cell (noise)
            while t < stop:
                c = self.cell_of_xy(lx + t * ca, ly + t * sa)
                if c is None:
                    break
                free.add(c)
                t += 0.05
        free -= hits
        return hits, free

    # ------------------------------------------------------------ 2 Hz decision
    def plan(self, x, y, yaw, scan, now):
        """scan = (ranges, angle_min, angle_inc, range_min, range_max)."""
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
        hits, free = self.scan_to_cells(x, y, yaw, *scan)
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
        if self.mode == "RACE" and (len(path) < 2 or math.hypot(x - self.goal[0], y - self.goal[1]) < 0.35):
            self.target = self.goal                        # last few cm: drive straight onto the goal
            self.last_mode = self.mode
            return
        if self.mode == "WAIT" or nxt == me and len(path) < 2:
            self.target = None
            return
        # aim a little ahead along the planned path for smoother driving
        aim = nxt
        look = self.cfg.get("lookahead_cells", 3)
        if len(path) >= 3 and path[0] == me:
            aim = path[min(look, len(path) - 1)]
        cx, cy = eg.center(aim)
        new = (cx * CELL, cy * CELL)
        # smooth small changes of direction (the grid planner can flip between two
        # equally good cells every tick; a real robot would waste time turning)
        if self.target is not None and self.mode == self.last_mode:
            a_old = math.atan2(self.target[1] - y, self.target[0] - x)
            a_new = math.atan2(new[1] - y, new[0] - x)
            if abs(wrap(a_new - a_old)) < math.radians(50):
                new = (0.5 * (new[0] + self.target[0]), 0.5 * (new[1] + self.target[1]))
        self.last_mode = self.mode
        self.target = new

    # ------------------------------------------------------------ 20 Hz control
    def control(self, x, y, yaw, front_min, now=None):
        """Returns (v, w). front_min = closest LiDAR range in +-30 deg in front."""
        if self.target is None:
            self.stuck_ref, self.stuck_since = None, None
            return 0.0, 0.0
        # stuck recovery: we keep asking to drive but do not move -> back up and turn
        if now is not None:
            if self.recover_until and now < self.recover_until:
                return -0.08, 1.2
            if self.last_v > 0.05:
                if self.stuck_ref is None or math.hypot(x - self.stuck_ref[0], y - self.stuck_ref[1]) > 0.03:
                    self.stuck_ref, self.stuck_since = (x, y), now
                elif self.stuck_since is not None and now - self.stuck_since > self.cfg.get("stuck_time", 2.0):
                    self.recover_until = now + 0.6
                    self.stuck_ref, self.stuck_since = None, None
                    return -0.08, 1.2
        vmax = self.cfg.get("max_lin", 0.20)
        wmax = self.cfg.get("max_ang", 1.5)
        tx, ty = self.target
        d = math.hypot(tx - x, ty - y)
        if d < 0.02:
            return 0.0, 0.0
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        w = max(-wmax, min(wmax, self.cfg.get("k_ang", 2.5) * err))
        if abs(err) > math.radians(75):
            v = 0.0                                        # turn on the spot first
        else:
            v = vmax * max(0.0, math.cos(err))
        if front_min < self.cfg.get("stop_dist", 0.13):
            v = 0.0                                        # safety: something right in front
        self.last_v = v
        return v, w
