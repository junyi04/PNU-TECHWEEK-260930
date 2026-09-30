"""Sensor-only mapping, frontier exploration, A*, and guarded path following.

Coordinates are relative to the initial robot pose; no world/map/target files,
GPS, recognition ground truth, or Supervisor API are used. This is a local
correlative mapper, not loop-closing SLAM. Low objects outside the lidar plane
and long-term drift remain limitations.
"""
import heapq
import math
from collections import deque

import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def shifted(array, dr, dc, fill=0):
    out = np.full_like(array, fill)
    h, w = array.shape
    sr, er = max(0, -dr), min(h, h - dr)
    sc, ec = max(0, -dc), min(w, w - dc)
    out[sr + dr:er + dr, sc + dc:ec + dc] = array[sr:er, sc:ec]
    return out


def inflate(mask, radius):
    out = mask.copy()
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if dr * dr + dc * dc <= radius * radius:
                out |= shifted(mask, dr, dc)
    return out


class PoseEstimator:
    def __init__(self):
        self.pose = np.zeros(3)
        self.previous = None
        self.distance = 0.0

    def update(self, wheels, gyro_z, dt):
        if self.previous is None:
            self.previous = np.array(wheels)
            return self.pose
        delta = (np.array(wheels) - self.previous) * 0.033
        self.previous = np.array(wheels)
        ds = float(delta.mean())
        wheel_turn = float((delta[1] - delta[0]) / 0.16)
        # Wheel yaw is biased during skid turns. Mixing it into a valid gyro
        # introduces systematic heading drift on every camera sweep.
        turn = wheel_turn if gyro_z is None else gyro_z * dt
        theta = self.pose[2]
        corrected = wrap(theta + turn)
        mid = theta + wrap(corrected - theta) / 2
        self.pose[:2] += ds * np.array([math.cos(mid), math.sin(mid)])
        self.pose[2] = corrected
        self.distance += abs(ds)
        return self.pose


class GridMap:
    def __init__(self, size=400, resolution=0.08):
        self.size = size
        self.resolution = resolution
        self.half = size * resolution / 2
        self.odds = np.zeros((size, size), dtype=np.int16)
        self.visits = np.zeros((size, size), dtype=np.uint16)
        self.free = np.zeros_like(self.odds, dtype=bool)
        self.safe = self.free.copy()
        self.clear_streak = np.zeros_like(self.odds, dtype=np.uint8)
        self.match_corrections = 0

    def cell(self, x, y):
        return (int(math.floor((y + self.half) / self.resolution)),
                int(math.floor((x + self.half) / self.resolution)))

    def xy(self, cell):
        r, c = cell
        return np.array([(c + 0.5) * self.resolution - self.half,
                         (r + 0.5) * self.resolution - self.half])

    def inside(self, cell):
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def indices(self, points):
        cols = np.floor((points[:, 0] + self.half) / self.resolution).astype(int)
        rows = np.floor((points[:, 1] + self.half) / self.resolution).astype(int)
        keep = (rows >= 0) & (cols >= 0) & (rows < self.size) & (cols < self.size)
        return rows[keep], cols[keep]

    @staticmethod
    def rays(ranges, maximum, stride=2):
        raw = np.asarray(ranges, dtype=float)[::stride]
        angles = math.pi - 2 * math.pi * np.arange(0, len(ranges), stride) / len(ranges)
        valid = (~np.isnan(raw)) & (raw > 0.12)
        angles, raw = angles[valid], raw[valid]
        hit = np.isfinite(raw) & (raw < maximum - 0.02)
        return angles, np.minimum(raw, maximum - 0.04), hit

    def match(self, pose, ranges, maximum):
        """Small correlative scan-to-map pose correction.

        Do not correct sparse/new scans, poor overlaps or stationary scans.
        Dynamic hits are diluted by the many static scan endpoints.
        """
        occupied = self.odds >= 4
        if np.count_nonzero(occupied) < 40:
            return
        angles, lengths, hit = self.rays(ranges, maximum, 4)
        if np.count_nonzero(hit) < 24:
            return
        angles, lengths = angles[hit] + pose[2], lengths[hit]
        origin = pose[:2] - 0.03 * np.array([math.cos(pose[2]), math.sin(pose[2])])
        endpoints = origin + np.column_stack((np.cos(angles), np.sin(angles))) * lengths[:, None]
        likelihood = occupied.astype(float)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            likelihood = np.maximum(likelihood, shifted(occupied, dr, dc) * 0.65)

        relative = endpoints - pose[:2]
        def score(offset, angle=0):
            cs, sn = math.cos(angle), math.sin(angle)
            rotated = relative @ np.array([[cs, sn], [-sn, cs]])
            r, c = self.indices(rotated + pose[:2] + offset)
            return float(likelihood[r, c].mean()) if len(r) else 0.0

        baseline = score((0, 0))
        if baseline < 0.45:
            return
        best, offset, rotation = baseline, (0, 0), 0
        for angle in (-0.012, 0, 0.012):
            for dx in (-0.025, 0, 0.025):
                for dy in (-0.025, 0, 0.025):
                    value = score((dx, dy), angle) - 0.6*math.hypot(dx, dy) - abs(angle)
                    if value > best + 0.045:
                        best, offset, rotation = value, (dx, dy), angle
        if offset != (0, 0) or rotation:
            pose[:2] += offset
            pose[2] = wrap(pose[2] + rotation)
            self.match_corrections += 1

    def update(self, pose, ranges, maximum):
        angles, lengths, hit = self.rays(ranges, maximum)
        theta = pose[2]
        origin = pose[:2] - 0.03 * np.array([math.cos(theta), math.sin(theta)])
        unit = np.column_stack((np.cos(angles + theta), np.sin(angles + theta)))
        steps = np.arange(0.04, maximum, self.resolution * 0.65)
        distances = lengths - np.where(hit, self.resolution, 0)
        valid = steps[None, :] < distances[:, None]
        points = origin + unit[:, None, :] * steps[None, :, None]
        r, c = self.indices(points[valid])
        free_ids = np.unique(r * self.size + c)
        flat = self.odds.ravel()
        flat[free_ids] = np.maximum(-20, flat[free_ids] - 2)
        r, c = self.indices(origin + unit[hit] * lengths[hit, None])
        hit_ids = np.unique(r * self.size + c)
        # Only repeated, directly observed free rays erase an old obstacle.
        # Unseen/occluded cells and current hit endpoints never age into free.
        clear_ids = np.setdiff1d(free_ids, hit_ids, assume_unique=True)
        streak = self.clear_streak.ravel()
        previous = streak[clear_ids].copy()
        streak[:] = 0
        streak[clear_ids] = np.minimum(previous, 2) + 1
        confirmed_clear = clear_ids[streak[clear_ids] >= 3]
        flat[confirmed_clear] = np.minimum(flat[confirmed_clear], -2)
        flat[hit_ids] = np.minimum(30, np.maximum(0, flat[hit_ids]) + 8)
        # The actually occupied robot footprint has been physically traversed.
        center = self.cell(*pose[:2])
        if self.inside(center):
            r, c = center
            self.odds[max(0, r-1):r+2, max(0, c-1):c+2] = -20
            self.visits[r, c] = min(65000, int(self.visits[r, c]) + 1)
        self.free = self.odds <= -2
        blocked = inflate(self.odds >= 4, math.ceil(0.19 / self.resolution))
        self.safe = self.free & ~blocked
        self.safe[[0, -1], :] = False
        self.safe[:, [0, -1]] = False

    def nearest_safe(self, cell, distance=4):
        if self.inside(cell) and self.safe[cell]:
            return cell
        candidates = [(dr * dr + dc * dc, (cell[0]+dr, cell[1]+dc))
                      for dr in range(-distance, distance+1)
                      for dc in range(-distance, distance+1)]
        for _, point in sorted(candidates):
            if self.inside(point) and self.safe[point]:
                return point
        return None

    def line_clear(self, a, b):
        steps = max(2, int(np.linalg.norm(b-a)/self.resolution*3)+1)
        previous = None
        for point in np.linspace(a, b, steps):
            cell = self.cell(*point)
            if not self.inside(cell) or not self.safe[cell]:
                return False
            if previous is not None and cell[0] != previous[0] and cell[1] != previous[1]:
                if not self.safe[previous[0], cell[1]] or not self.safe[cell[0], previous[1]]:
                    return False
            previous = cell
        return True

    def frontier_goals(self, pose, blacklist, now):
        self.frontier_gain = {}
        unknown = self.odds == 0
        boundary = self.safe & (shifted(unknown, 1, 0) | shifted(unknown, -1, 0) |
                                shifted(unknown, 0, 1) | shifted(unknown, 0, -1))
        remaining = set(map(tuple, np.argwhere(boundary)))
        goals = []
        while remaining:
            seed = remaining.pop()
            group, queue = [seed], deque([seed])
            while queue:
                r, c = queue.popleft()
                for dr, dc in NEIGHBORS:
                    point = (r+dr, c+dc)
                    if point in remaining:
                        remaining.remove(point)
                        group.append(point)
                        queue.append(point)
            if len(group) < 4:
                continue
            centroid = np.mean(group, axis=0)
            point = min(group, key=lambda p: np.sum((np.array(p)-centroid)**2))
            xy = self.xy(point)
            distance = float(np.linalg.norm(xy-pose[:2]))
            if distance < 0.5 or any(now < until and np.linalg.norm(xy-old) < 0.65
                                     for old, until in blacklist):
                continue
            goals.append((distance - min(1.0, 0.03 * len(group)), point))
            self.frontier_gain[point] = len(group)
        return [p for _, p in sorted(goals)[:16]]


NEIGHBORS = ((-1, 0), (1, 0), (0, -1), (0, 1),
             (-1, -1), (-1, 1), (1, -1), (1, 1))


def path_cost(path, resolution=1.):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:])) * resolution


def astar(safe, start, goal):
    """8-neighbor A* with Euclidean admissible heuristic, no corner cutting."""
    return astar_to_any(safe, start, [goal])


def astar_to_any(safe, start, goals):
    """One search finds the shortest route to ANY acceptable destination."""
    h, w = safe.shape
    if start is None or not (0 <= start[0] < h and 0 <= start[1] < w) or not safe[start]:
        return []
    goals = {g for g in goals if g is not None and 0 <= g[0] < h and 0 <= g[1] < w and safe[g]}
    if not goals:
        return []
    def heuristic(point):
        return min(math.dist(point, goal) for goal in goals)
    queue = [(heuristic(start), 0.0, start)]
    costs, parent = {start: 0.0}, {}
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost > costs[current]:
            continue
        if current in goals:
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            return path[::-1]
        for dr, dc in NEIGHBORS:
            nr, nc = current[0]+dr, current[1]+dc
            if not (0 <= nr < h and 0 <= nc < w) or not safe[nr, nc]:
                continue
            if dr and dc and (not safe[current[0]+dr, current[1]] or
                              not safe[current[0], current[1]+dc]):
                continue
            point = (nr, nc)
            value = cost + (math.sqrt(2) if dr and dc else 1)
            if value < costs.get(point, math.inf):
                costs[point], parent[point] = value, current
                heapq.heappush(queue, (value + heuristic(point), value, point))
    return []


def local_command(pose, waypoint, ranges, maximum):
    """Short-horizon velocity sampling against live lidar, not full DWA.

    Rotate first for large heading errors. Collision envelopes include a
    conservative braking allowance, but cannot see below the lidar plane.
    """
    delta = waypoint - pose[:2]
    error = wrap(math.atan2(delta[1], delta[0]) - pose[2])
    desired = max(-0.8, min(0.8, 2.0 * error))
    angles, lengths, hit = GridMap.rays(ranges, maximum, 1)
    obstacles = np.column_stack((lengths[hit]*np.cos(angles[hit])-0.03,
                                 lengths[hit]*np.sin(angles[hit])))
    target = np.array([math.cos(pose[2])*delta[0]+math.sin(pose[2])*delta[1],
                       -math.sin(pose[2])*delta[0]+math.cos(pose[2])*delta[1]])
    best, command = -math.inf, (0.0, 0.0)
    speeds = (0.0,) if abs(error) > 0.55 else (0.0, 0.045, 0.085, 0.12)
    turns = sorted(set((desired, max(-0.8, desired-0.3), min(0.8, desired+0.3), 0.0)))
    for v in speeds:
        for omega in turns:
            if v == 0 and abs(omega) < 0.02:
                continue
            x = y = heading = 0.0
            clearance = 3.5
            for _ in range(10):
                x += v * math.cos(heading + omega*0.05) * 0.1
                y += v * math.sin(heading + omega*0.05) * 0.1
                heading += omega*0.1
                if len(obstacles):
                    clearance = min(clearance, float(np.min(np.hypot(obstacles[:, 0]-x,
                                                                     obstacles[:, 1]-y))))
            if clearance < 0.155 + v*v/(2*0.3):
                continue
            progress = float(np.linalg.norm(target)-np.linalg.norm(target-[x, y]))
            score = 4*progress - 0.7*abs(wrap(error-heading)) + 0.12*min(clearance, 0.45)
            if score > best:
                best, command = score, (v, omega)
    return command


class Navigator:
    def __init__(self, explore_seconds=math.inf):
        self.localization = PoseEstimator()
        self.grid = GridMap()
        self.mode = "MANUAL"
        self.reason = "ready"
        self.limit = explore_seconds
        self.started = 0.0
        self.last_map = -1.0
        self.last_match_pose = np.zeros(3)
        self.path = []
        self.goal = None
        self.blacklist = []
        self.last_plan = -100.0
        self.progress_time = 0.0
        self.progress_position = np.zeros(2)
        self.trace = []
        self.frontiers_reached = 0
        self.no_route_since = None
        self.last_real_progress = 0.0
        self.target_xy = None
        self.return_plan_failure = None
        self.return_route_kind = None
        self.obstacle_wait_since = None
        self.obstacle_wait_cooldown = 0.
        self.return_recovery_seen = set()

    @property
    def pose(self):
        return self.localization.pose

    def set_mode(self, mode, now):
        self.mode = mode
        self.reason = "requested"
        self.path = []
        self.goal = None
        self.last_plan = -100
        self.progress_time = now
        self.last_real_progress = now
        self.progress_position = self.pose[:2].copy()
        self.no_route_since = None
        self.obstacle_wait_since = None
        self.obstacle_wait_cooldown = now
        if mode == "EXPLORE":
            self.started = now

    def observe(self, wheels, gyro, ranges, maximum, now, dt):
        self.localization.update(wheels, gyro, dt)
        if now-self.last_map < 0.256:
            return
        if (np.linalg.norm(self.pose[:2]-self.last_match_pose[:2]) > 0.06 or
                abs(wrap(self.pose[2]-self.last_match_pose[2])) > 0.12):
            self.grid.match(self.pose, ranges, maximum)
            self.last_match_pose = self.pose.copy()
        self.grid.update(self.pose, ranges, maximum)
        self.last_map = now
        if not self.trace or math.dist(self.trace[-1], self.pose[:2]) > 0.07:
            self.trace.append(self.pose[:2].tolist())

    def go_to(self, xy, now):
        self.set_mode('TARGET', now)
        self.target_xy = np.array(xy, dtype=float)
        self.goal = self.grid.nearest_safe(self.grid.cell(*xy), distance=1)

    def choose_frontier(self, start, candidates):
        best = None
        for target in candidates:
            path = astar(self.grid.safe, start, target)
            if not path:
                continue
            delta = self.grid.xy(path[min(3, len(path)-1)])-self.pose[:2]
            turn = abs(wrap(math.atan2(delta[1], delta[0])-self.pose[2]))
            gain = getattr(self.grid, 'frontier_gain', {}).get(target, 0)
            score = path_cost(path, self.grid.resolution) + .12*turn - min(.8, .02*gain)
            if best is None or score < best[0]:
                best = score, target, path
        return (best[1], best[2]) if best else (None, [])

    def plan_home(self, start):
        """Try reachable home cells, not just the single geometrically nearest one.

        All endpoints must satisfy the existing arrival radius. Never clear
        obstacles or unknown cells to manufacture a return route.
        """
        previous_goal = self.goal if self.return_route_kind == 'recorded_backtrack' else None
        self.path, self.goal = [], None
        self.return_route_kind = None
        if start is None:
            self.return_plan_failure = 'robot_has_no_safe_start_cell'
            return
        row, col = self.grid.cell(0, 0)
        candidates = []
        for dr in range(-3, 4):
            for dc in range(-3, 4):
                cell = row+dr, col+dc
                if self.grid.inside(cell) and self.grid.safe[cell]:
                    distance = float(np.linalg.norm(self.grid.xy(cell)))
                    if distance < .16:
                        candidates.append((distance, cell))
        self.return_plan_failure = 'home_region_blocked' if not candidates else 'home_region_disconnected'
        path = astar_to_any(self.grid.safe, start, [cell for _, cell in candidates])
        if path:
            self.path, self.goal = path, path[-1]
            self.return_plan_failure = None
            self.return_route_kind = 'home'
            return
        if previous_goal is not None and np.linalg.norm(self.grid.xy(previous_goal)-self.pose[:2]) >= .22:
            path = astar(self.grid.safe, start, previous_goal)
            if path:
                self.path, self.goal = path, previous_goal
                self.return_route_kind = 'recorded_backtrack'
                return
        # A long run can temporarily obscure home in the map. Get closer via
        # an earlier observed waypoint, but ONLY along currently safe A* cells.
        # Historical traversal alone is never used as collision clearance.
        current_distance = float(np.linalg.norm(self.pose[:2]))
        seen, recovery = set(), []
        for xy in self.trace:
            cell = self.grid.cell(*xy)
            if cell in seen or not self.grid.inside(cell) or not self.grid.safe[cell]:
                continue
            seen.add(cell)
            distance = float(np.linalg.norm(self.grid.xy(cell)))
            if distance >= current_distance-.30 or np.linalg.norm(self.grid.xy(cell)-self.pose[:2]) < .45:
                continue
            recovery.append((distance, cell))
        # Bound planning cost and avoid testing dozens of almost identical points.
        spaced = []
        for _, cell in sorted(recovery):
            if any(math.dist(cell, old) < 4 for old in spaced):
                continue
            spaced.append(cell)
            path = astar(self.grid.safe, start, cell)
            if path:
                self.path, self.goal = path, cell
                self.return_route_kind = 'recorded_waypoint'
                return
            if len(spaced) >= 24:
                break
        # A U-shaped route can require moving AWAY from home first. Restricting
        # recovery to smaller Euclidean distance traps the robot in such rooms.
        checked = 0
        for xy in reversed(self.trace):
            cell = self.grid.cell(*xy)
            if (cell in self.return_recovery_seen or not self.grid.inside(cell)
                    or not self.grid.safe[cell] or math.dist(xy, self.pose[:2]) < .5):
                continue
            checked += 1
            path = astar(self.grid.safe, start, cell)
            if path:
                self.path, self.goal = path, cell
                self.return_recovery_seen.add(cell)
                self.return_route_kind = 'recorded_backtrack'
                return
            if checked >= 24:
                break

    def command(self, now, ranges, maximum):
        if self.mode not in ("EXPLORE", "RETURN", "TARGET"):
            return 0.0, 0.0
        if self.mode == "EXPLORE" and now-self.started >= self.limit:
            self.set_mode("RETURN", now)
            self.reason = "exploration_time_limit"
        if self.mode == "RETURN" and np.linalg.norm(self.pose[:2]) < 0.16:
            self.mode, self.reason = "HOME", "within_16cm_of_estimated_start"
            self.path = []
            return 0.0, 0.0
        if self.mode == 'TARGET' and np.linalg.norm(self.pose[:2]-self.target_xy) < .10:
            self.mode, self.reason = 'ARRIVED', 'waypoint_reached'
            self.path = []
            return 0.0, 0.0
        if not self.grid.inside(self.grid.cell(*self.pose[:2])):
            self.mode, self.reason = "STOPPED", "map_boundary"
            return 0.0, 0.0
        # Keep the destination while a newly blocked near route may clear.
        # Bounded waiting applies equally to search, approach and return.
        if self.path:
            closest = min(range(min(len(self.path), 15)),
                          key=lambda i: np.linalg.norm(self.grid.xy(self.path[i])-self.pose[:2]))
            nearby = self.path[closest:closest+13]
            blocked = any(not self.grid.safe[p] for p in nearby)
            if blocked and (self.obstacle_wait_since is not None or now >= self.obstacle_wait_cooldown):
                if self.obstacle_wait_since is None:
                    self.obstacle_wait_since = now
                if now-self.obstacle_wait_since < 3.:
                    self.reason = 'waiting_for_obstacle'
                    return 0., 0.
                self.obstacle_wait_since = None
                self.obstacle_wait_cooldown = now+6.
                self.path = []
                self.last_plan = -100.
            elif not blocked and self.obstacle_wait_since is not None:
                self.obstacle_wait_since = None
                self.last_plan = now
                self.reason = 'obstacle_cleared_resume'
        if np.linalg.norm(self.pose[:2]-self.progress_position) > 0.10:
            self.progress_time = now
            self.last_real_progress = now
            self.progress_position = self.pose[:2].copy()
        if self.mode in ('RETURN', 'TARGET') and now-self.last_real_progress > 45:
            self.mode, self.reason = "STOPPED", "route_stalled"
            return 0.0, 0.0
        if (self.mode != 'TARGET' and not (self.mode == 'RETURN' and self.return_route_kind == 'home')
                and self.goal is not None and np.linalg.norm(self.pose[:2]-self.grid.xy(self.goal)) < 0.22):
            if self.mode == "EXPLORE":
                self.frontiers_reached += 1
                self.blacklist.append((self.grid.xy(self.goal), now+90))
            self.path, self.goal = [], None
        if now-self.progress_time > 14:
            if self.goal is not None and self.mode == "EXPLORE":
                self.blacklist.append((self.grid.xy(self.goal), now+90))
                self.goal = None
            self.path = []
            self.progress_time = now
            self.reason = "stuck_replan"
        # Validate the current route cheaply; don't rebuild an unchanged A*
        # route every two seconds. Refresh periodically to discover shortcuts.
        route_invalid = bool(self.path) and any(not self.grid.safe[p] for p in self.path)
        if now-self.last_plan > 6.0 or not self.path or route_invalid:
            if not self.path and now-self.last_plan < 0.8:
                return 0.0, 0.0
            self.last_plan = now
            start = self.grid.nearest_safe(self.grid.cell(*self.pose[:2]))
            if self.mode == "RETURN":
                self.plan_home(start)
            elif self.mode == 'TARGET':
                self.goal = self.grid.nearest_safe(self.grid.cell(*self.target_xy), distance=1)
            if self.mode != 'RETURN' and self.goal is not None:
                self.path = astar(self.grid.safe, start, self.goal)
            if not self.path and self.mode == "EXPLORE":
                if self.goal is not None:
                    self.blacklist.append((self.grid.xy(self.goal), now+45))
                self.goal = None
                candidates = self.grid.frontier_goals(self.pose, self.blacklist, now)
                self.goal, self.path = self.choose_frontier(start, candidates)
                if self.path:
                    self.reason = "frontier_selected_by_route_cost"
                if not self.path:
                    self.set_mode("RETURN", now)
                    self.reason = "no_reachable_frontiers"
                    return 0.0, 0.0
            if not self.path:
                self.reason = "path_blocked_waiting"
                if self.no_route_since is None:
                    self.no_route_since = now
                if now-self.no_route_since > 20:
                    self.mode, self.reason = "STOPPED", "path_unavailable"
                return 0.0, 0.0
            self.no_route_since = None
        if not self.path:
            return 0.0, 0.0
        # Advance along the path without cutting through unobserved corners.
        closest = min(range(min(len(self.path), 15)),
                      key=lambda i: np.linalg.norm(self.grid.xy(self.path[i])-self.pose[:2]))
        self.path = self.path[closest:]
        index = 0
        for i in range(1, min(len(self.path), 13)):
            if np.linalg.norm(self.grid.xy(self.path[i])-self.pose[:2]) > 0.60:
                break
            if not self.grid.line_clear(self.pose[:2], self.grid.xy(self.path[i])):
                break
            index = i
        waypoint = self.grid.xy(self.path[index])
        if index == 0 and len(self.path) > 1:
            self.path = []
            self.reason = 'no_safe_lookahead'
            return 0., 0.
        if not all(self.grid.safe[p] for p in self.path[:max(2, index+1)]):
            self.path = []
            self.reason = "path_changed"
            return 0.0, 0.0
        return local_command(self.pose, waypoint, ranges, maximum)

    def telemetry(self):
        return {"pose": self.pose.tolist(), "nav_mode": self.mode, "reason": self.reason,
                "home_distance_m": float(np.linalg.norm(self.pose[:2])),
                "return_plan_failure": self.return_plan_failure,
                "return_route_kind": self.return_route_kind,
                "waiting_for_obstacle": self.obstacle_wait_since is not None,
                "mapped_m2": float(np.count_nonzero(self.grid.odds) * self.grid.resolution**2),
                "distance_m": self.localization.distance,
                "frontiers_reached": self.frontiers_reached,
                "scan_corrections": self.grid.match_corrections,
                "goal": self.grid.xy(self.goal).tolist() if self.goal else None}

    def render(self, display):
        """Auto-cropped map: gray unknown, white free, dark obstacle, blue route."""
        known = np.argwhere(self.grid.odds != 0)
        if not len(known):
            return
        low = np.maximum(known.min(axis=0)-4, 0)
        high = np.minimum(known.max(axis=0)+5, self.grid.size)
        size = max(high-low)
        low = np.maximum(0, np.minimum(low, self.grid.size-size))
        width, height = display.getWidth(), display.getHeight()
        rr = np.minimum(low[0]+np.arange(height-40)*size//(height-40), self.grid.size-1)
        cc = np.minimum(low[1]+np.arange(width)*size//width, self.grid.size-1)
        values = self.grid.odds[rr[:, None], cc[None, :]][::-1]
        pixels = np.full((height-40, width, 3), 110, dtype=np.uint8)
        pixels[values <= -2] = (238, 240, 242)
        pixels[values >= 4] = (30, 36, 45)
        image = display.imageNew(pixels.tobytes(), display.RGB, width, height-40)
        display.imagePaste(image, 0, 40, False)
        display.imageDelete(image)

        def screen(xy):
            r, c = self.grid.cell(*xy)
            return int((c-low[1])*width/size), int(height-1-(r-low[0])*(height-40)/size)

        display.setColor(0xA78BFA)
        for a, b in zip(self.trace, self.trace[1:]):
            display.drawLine(*screen(a), *screen(b))
        display.setColor(0x168BDA)
        for a, b in zip(self.path, self.path[1:]):
            display.drawLine(*screen(self.grid.xy(a)), *screen(self.grid.xy(b)))
        display.setColor(0xFF8800)
        for target in getattr(self, 'targets', []):
            status = target.get('status', 'candidate')
            display.setColor(0x16A34A if status == 'visited' else
                             0x888888 if status == 'ambiguous_revisit' else 0xFF8800)
            tx, ty = screen(target['position'])
            display.drawOval(tx, ty, 7, 7)
            label = 'visited' if status == 'visited' else 'recheck' if status == 'ambiguous_revisit' else 'apple'
            display.drawText(f"{label} {target['id']}", tx+8, ty)
        display.setColor(0x16A34A)
        x, y = screen((0, 0))
        display.fillOval(x, y, 5, 5)
        display.setColor(0xEF4444)
        x, y = screen(self.pose[:2])
        display.fillOval(x, y, 5, 5)
        tip = self.pose[:2] + 0.3*np.array([math.cos(self.pose[2]), math.sin(self.pose[2])])
        display.drawLine(x, y, *screen(tip))
        display.setColor(0x17202C)
        display.fillRectangle(0, 0, width, 40)
        display.setColor(0xFFFFFF)
        display.setFont("Arial", 13, True)
        display.drawText(getattr(self, 'mission_label', f"{self.mode} | N explore H home SPACE stop"), 6, 3)
        display.drawText(f"x={self.pose[0]:.2f} y={self.pose[1]:.2f}  {self.reason}", 6, 21)
