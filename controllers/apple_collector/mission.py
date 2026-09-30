"""Visit two visually observed red targets and return; no world-file access.

Arrival thresholds are MVP choices, not official competition scoring rules.
"""
import math
import numpy as np
from navigation import astar, wrap, path_cost


class Mission:
    def __init__(self, navigator):
        self.nav = navigator
        self.state = 'IDLE'
        self.reason = 'ready'
        self.visited = []
        self.active = None
        self.deferred = {}
        self.attempts = {}
        self.scanned = []
        self.scan_angle = 0.
        self.scan_heading = 0.
        self.since = 0.
        self.started = None
        self.verify_frames = []
        self.exhaustions = 0
        self.terminal_reason = None
        self.last_selection = -100.
        self.vision_wait_since = None
        self.resume_goal = None
        self.scan_count = 0
        self.recovery_count = 0

    def recover_return(self, now):
        self.state, self.reason = 'RECOVER_RETURN', 'refresh_return_map'
        self.since = now
        self.recovery_count += 1
        self.nav.set_mode('MANUAL', now)

    def retry_search(self, now):
        # Revisit known areas instead of treating a temporarily exhausted map
        # or three unsuccessful approaches as the end of the mission.
        self.exhaustions += 1
        self.nav.blacklist = []
        self.scanned = []
        self.attempts = {}
        self.begin_scan(now)
        self.reason = 'retry_unfinished_search'

    def pause(self, now):
        self.state, self.reason = 'PAUSED', 'operator_pause'
        self.nav.set_mode('MANUAL', now)

    def start(self, now):
        if self.started is None:
            self.started = now
        if len(self.visited) >= 2:
            self.return_home(now, 'two_targets_visited')
        else:
            self.begin_scan(now)

    def return_home(self, now, reason):
        self.state, self.reason = 'RETURN', reason
        self.terminal_reason = reason
        self.nav.set_mode('RETURN', now)

    def begin_scan(self, now):
        # Keep a selected frontier across an observation stop. Do not sweep
        # the same location again just because another state requested it.
        self.resume_goal = self.nav.goal if self.nav.mode == 'EXPLORE' else None
        if any(math.dist(self.nav.pose[:2], p) < .75 for p in self.scanned):
            self.resume_search(now, 'recently_observed_location')
            return
        self.state, self.reason = 'SCAN', 'look_around'
        self.scan_count += 1
        self.since, self.scan_angle = now, 0.
        self.scan_heading = self.nav.pose[2]
        self.nav.set_mode('MANUAL', now)

    def resume_search(self, now, reason='sweep_complete'):
        self.state, self.reason = 'SEARCH', reason
        self.nav.set_mode('EXPLORE', now)
        self.nav.goal = self.resume_goal
        self.resume_goal = None

    def remaining(self, targets):
        return [t for t in targets if not any(t['id'] == v['id'] or
                math.dist(t['position'], v['position']) < .30 for v in self.visited)]

    def approach_point(self, target):
        grid = self.nav.grid
        center = np.array(target['position'])
        start = grid.nearest_safe(grid.cell(*self.nav.pose[:2]))
        toward = math.atan2(self.nav.pose[1]-center[1], self.nav.pose[0]-center[0])
        candidates = []
        for radius in (.40, .36):
            for offset in (0, .5, -.5, 1., -1., 1.6, -1.6, math.pi):
                xy = center + radius*np.array([math.cos(toward+offset), math.sin(toward+offset)])
                cell = grid.cell(*xy)
                if not grid.inside(cell) or not grid.safe[cell]:
                    continue
                # Do not approach a visual estimate through a mapped wall.
                sight = [grid.cell(*p) for p in np.linspace(xy, center, 12)]
                if any(not grid.inside(c) or grid.odds[c] >= 4 for c in sight):
                    continue
                path = astar(grid.safe, start, cell)
                if path:
                    candidates.append((path_cost(path, grid.resolution), grid.xy(cell)))
        return min(candidates, key=lambda p: p[0])[1] if candidates else None

    def select_target(self, targets, now):
        if now-self.last_selection < 1.:
            return False
        self.last_selection = now
        options = sorted(self.remaining(targets), key=lambda t: math.dist(t['position'], self.nav.pose[:2]))
        choices = []
        start = self.nav.grid.nearest_safe(self.nav.grid.cell(*self.nav.pose[:2]))
        for target in options:
            if self.deferred.get(target['id'], 0) > now or self.attempts.get(target['id'], 0) >= 3:
                continue
            point = self.approach_point(target)
            if point is None:
                self.deferred[target['id']] = now+20
                continue
            route = astar(self.nav.grid.safe, start, self.nav.grid.cell(*point))
            if route:
                choices.append((path_cost(route, self.nav.grid.resolution), target, point))
        if choices:
            _, target, point = min(choices, key=lambda c: c[0])
            self.active = dict(target)
            self.attempts[target['id']] = self.attempts.get(target['id'], 0)+1
            self.state, self.reason, self.since = 'APPROACH', 'target_selected', now
            self.nav.go_to(point, now)
            return True
        return False

    def abandon_target(self, now, reason):
        if self.active:
            self.deferred[self.active['id']] = now+30
        self.active = None
        self.state, self.reason = 'SEARCH', reason
        self.nav.set_mode('EXPLORE', now)

    def coverage_point(self):
        """Look into already mapped areas not covered by a camera sweep yet."""
        grid = self.nav.grid
        cells = np.argwhere(grid.safe)[::12]
        candidates = []
        for cell in cells:
            xy = grid.xy(cell)
            if self.scanned and min(math.dist(xy, p) for p in self.scanned) < 1.3:
                continue
            distance = math.dist(xy, self.nav.pose[:2])
            if distance < .5:
                continue
            candidates.append((distance, tuple(cell)))
        start = grid.nearest_safe(grid.cell(*self.nav.pose[:2]))
        for _, cell in sorted(candidates)[:40]:
            if astar(grid.safe, start, cell):
                return grid.xy(cell)
        return None

    def protect_targets(self, targets):
        # Low apples are below the LiDAR plane. Keep the chassis off known fruit.
        grid = self.nav.grid
        for t in targets:
            r, c = grid.cell(*t['position'])
            for dr in range(-3, 4):
                for dc in range(-3, 4):
                    cell = (r+dr, c+dc)
                    if dr*dr+dc*dc <= 9 and grid.inside(cell):
                        grid.safe[cell] = False

    def command(self, now, ranges, maximum, targets, vision_status):
        if self.state in ('IDLE', 'PAUSED', 'SUCCEEDED', 'INCOMPLETE', 'FAILED'):
            return 0., 0.
        if vision_status == 'error' and self.state not in ('RETURN', 'RECOVER_RETURN'):
            self.return_home(now, 'vision_unavailable')
        if vision_status in ('loading', 'stale') and self.state not in ('RETURN', 'RECOVER_RETURN'):
            if self.vision_wait_since is None:
                self.vision_wait_since = now
            if now-self.vision_wait_since < 30:
                return 0., 0.
            self.return_home(now, 'vision_timeout')
        else:
            self.vision_wait_since = None
        if self.started is not None and now-self.started >= self.nav.limit and self.state not in ('RETURN', 'RECOVER_RETURN'):
            self.return_home(now, 'explicit_time_limit')
        self.protect_targets(targets)
        if self.state == 'RECOVER_RETURN':
            if now-self.since >= 8:
                self.state, self.reason = 'RETURN', 'retry_return_route'
                self.nav.set_mode('RETURN', now)
                return 0., 0.
            # Only rotate with clearance in every direction. A blocked robot
            # keeps sensing and replanning; never drive blindly through a wall.
            clearance = min((r for r in ranges if math.isfinite(r)), default=maximum)
            return (0., .25) if clearance > .24 else (0., 0.)
        if self.state == 'RETURN':
            command = self.nav.command(now, ranges, maximum)
            if self.nav.mode == 'HOME':
                self.state = 'SUCCEEDED' if len(self.visited) == 2 and self.terminal_reason == 'two_targets_visited' else 'INCOMPLETE'
                self.reason = self.terminal_reason
            elif self.nav.mode == 'STOPPED':
                self.recover_return(now)
                return 0., 0.
            return command
        if self.state in ('SEARCH', 'SCAN', 'COVERAGE') and self.select_target(targets, now):
            return 0., 0.
        if self.state == 'SCAN':
            self.scan_angle += abs(wrap(self.nav.pose[2]-self.scan_heading))
            self.scan_heading = self.nav.pose[2]
            if self.scan_angle >= 2*math.pi or now-self.since > 28:
                self.scanned.append(self.nav.pose[:2].tolist())
                self.resume_search(now)
                return 0., 0.
            return (0., 0.) if now-self.since < 3 else (0., .35)
        if self.state == 'APPROACH':
            fresh = next((t for t in targets if t['id'] == self.active['id']), None)
            if fresh:
                self.active = dict(fresh)
            if self.nav.mode == 'ARRIVED' or math.dist(self.nav.pose[:2], self.active['position']) < .46:
                self.state, self.reason, self.since = 'VERIFY', 'confirm_at_target', now
                self.verify_frames = []
                self.nav.set_mode('MANUAL', now)
                return 0., 0.
            command = self.nav.command(now, ranges, maximum)
            if self.nav.mode == 'STOPPED' or now-self.since > 100:
                self.abandon_target(now, 'target_unreachable')
            return command
        if self.state == 'VERIFY':
            target = next((t for t in targets if t['id'] == self.active['id']), self.active)
            delta = np.array(target['position'])-self.nav.pose[:2]
            angle = wrap(math.atan2(delta[1], delta[0])-self.nav.pose[2])
            if now-self.since > 15:
                self.abandon_target(now, 'close_view_unconfirmed')
                return 0., 0.
            if abs(angle) > .10:
                return 0., max(-.4, min(.4, 1.4*angle))
            # Require NEW close camera observations after arrival, not a cached track.
            if (np.linalg.norm(delta) <= .55 and target['last'] >= self.since and
                    now-target['last'] < 2.5 and target.get('observed_distance', 999) <= .55 and
                    target['last'] not in self.verify_frames):
                self.verify_frames.append(target['last'])
            if len(self.verify_frames) >= 3 and self.verify_frames[-1]-self.verify_frames[0] >= .3:
                if not self.remaining([target]):
                    self.abandon_target(now, 'already_visited_target')
                    return 0., 0.
                self.visited.append(dict(id=target['id'], position=target['position'], time=now,
                                         source=target.get('source'), observed_distance=target['observed_distance']))
                self.active = None
                if len(self.visited) >= 2:
                    self.return_home(now, 'two_targets_visited')
                else:
                    self.begin_scan(now)
            return 0., 0.
        if self.state == 'COVERAGE':
            command = self.nav.command(now, ranges, maximum)
            if self.nav.mode == 'ARRIVED':
                self.begin_scan(now)
            elif self.nav.mode == 'STOPPED':
                self.scanned.append(self.nav.target_xy.tolist())
                self.begin_scan(now)
            return command
        if self.state == 'SEARCH':
            reached_before = self.nav.frontiers_reached
            command = self.nav.command(now, ranges, maximum)
            if self.nav.mode == 'EXPLORE' and self.nav.frontiers_reached > reached_before:
                self.begin_scan(now)
                return 0., 0.
            if self.nav.mode == 'RETURN' and self.nav.reason == 'no_reachable_frontiers':
                point = self.coverage_point()
                if point is not None:
                    self.state, self.reason = 'COVERAGE', 'inspect_mapped_area'
                    self.nav.go_to(point, now)
                else:
                    self.retry_search(now)
            elif self.nav.mode == 'STOPPED':
                self.retry_search(now)
            return command
        return 0., 0.

    def telemetry(self):
        return dict(mission_state=self.state, mission_reason=self.reason,
                    recovery_count=self.recovery_count,
                    scan_count=self.scan_count,
                    visited_count=len(self.visited), visited=self.visited,
                    active_target=self.active['id'] if self.active else None)
