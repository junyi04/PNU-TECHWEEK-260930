import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'controllers/apple_collector'))
from navigation import Navigator
from mission import Mission


class MissionTests(unittest.TestCase):
    def test_two_visits_force_return_from_any_autonomous_state(self):
        for state in ('IDLE', 'SCAN', 'SEARCH', 'APPROACH', 'VERIFY', 'COVERAGE', 'FAILED', 'INCOMPLETE'):
            with self.subTest(state=state):
                nav = Navigator()
                nav.grid.safe[:] = True
                nav.pose[:] = [2, 0, math.pi]
                mission = Mission(nav)
                mission.state = state
                mission.visited = [self.target(identity=1), self.target(identity=2, xy=(2, 2))]
                mission.active = self.target(identity=3)
                mission.command(1, self.scan, 3.5, [], 'stale')
                self.assertEqual(mission.state, 'RETURN')
                self.assertEqual(nav.mode, 'RETURN')
                self.assertIsNone(mission.active)
                self.assertEqual(mission.terminal_reason, 'two_targets_visited')

    def test_two_visits_do_not_override_operator_pause(self):
        self.m.visited = [self.target(identity=1), self.target(identity=2)]
        self.m.pause(0)
        self.assertEqual(self.m.command(1, self.scan, 3.5, [], 'running'), (0., 0.))
        self.assertEqual(self.m.state, 'PAUSED')

    def test_approach_search_matches_individual_routes_and_reuses_result(self):
        from unittest.mock import patch
        import mission
        from navigation import astar, astar_to_any, path_cost
        target = self.target(xy=(2, .5))
        captured = []
        def search(safe, start, goals):
            goals = list(goals)
            captured.extend(goals)
            return astar_to_any(safe, start, goals)
        with patch.object(mission, 'astar_to_any', side_effect=search) as planner:
            self.assertTrue(self.m.select_target([target], 1))
            self.assertEqual(planner.call_count, 1)
        start = self.nav.grid.cell(0, 0)
        reference = min(path_cost(astar(self.nav.grid.safe, start, g)) for g in captured)
        self.assertAlmostEqual(path_cost(self.nav.path), reference)
        self.assertEqual(self.nav.last_plan, 1)
        self.assertEqual(self.nav.path[-1], self.nav.goal)

    def test_duplicate_target_cannot_be_counted_at_final_verification(self):
        self.m.visited = [self.target(identity=1)]
        self.m.state, self.m.since = 'VERIFY', 1
        self.m.active = self.target(identity=2)
        for now in (2, 2.4, 2.8):
            self.m.command(now, self.scan, 3.5, [self.target(now, identity=2)], 'running')
        self.assertEqual(len(self.m.visited), 1)
        self.assertNotEqual(self.m.state, 'RETURN')

    def setUp(self):
        self.nav = Navigator()
        self.nav.grid.safe[:] = True
        self.nav.grid.odds[:] = -10
        self.m = Mission(self.nav)
        self.scan = [3.]*360

    def target(self, time=1, identity=1, xy=(.43, 0)):
        return dict(id=identity, position=list(xy), last=time, hits=5,
                    observed_distance=.43, source='appearance_candidate')

    def test_default_has_no_180_second_deadline(self):
        self.assertTrue(math.isinf(self.nav.limit))
        self.m.start(0)
        self.m.command(181, self.scan, 3.5, [], 'running')
        self.assertNotEqual(self.m.state, 'RETURN')

    def test_cached_detection_cannot_complete_visit(self):
        self.m.state, self.m.since = 'VERIFY', 5
        self.m.active = self.target(1)
        for now in (5, 5.5, 6):
            self.m.command(now, self.scan, 3.5, [self.target(1)], 'running')
        self.assertEqual(self.m.visited, [])

    def test_new_close_views_then_second_target_and_return(self):
        for identity, xy in [(1, (.43, 0)), (2, (0, .43))]:
            self.m.state, self.m.since = 'VERIFY', 5*identity
            self.m.active = self.target(5*identity, identity, xy)
            self.m.verify_frames = []
            self.nav.pose[2] = math.atan2(xy[1], xy[0])
            for now in (5*identity+.1, 5*identity+.3, 5*identity+.6):
                target = self.target(now, identity, xy)
                target['distinct_from'] = [3-identity]
                self.m.command(now, self.scan, 3.5, [target], 'running')
        self.assertEqual(len(self.m.visited), 2)
        self.assertEqual(self.m.state, 'RETURN')
        self.m.command(12, self.scan, 3.5, [], 'running')
        self.assertEqual(self.m.state, 'SUCCEEDED')

    def test_home_without_two_visits_is_not_success(self):
        self.m.return_home(0, 'targets_not_found')
        self.m.command(1, self.scan, 3.5, [], 'running')
        self.assertEqual(self.m.state, 'INCOMPLETE')

    def test_vision_failure_and_operator_pause(self):
        self.m.start(0)
        self.m.command(1, self.scan, 3.5, [], 'error')
        self.assertEqual(self.m.state, 'INCOMPLETE')
        self.m.pause(2)
        self.assertEqual(self.m.command(3, self.scan, 3.5, [], 'running'), (0, 0))

    def test_unreachable_target_does_not_cut_wall(self):
        self.nav.grid.safe[:] = False
        self.assertIsNone(self.m.approach_point(self.target()))

    def test_target_exclusion_and_nearby_distinct_target(self):
        self.m.visited = [self.target()]
        separate = self.target(identity=2, xy=(.43, .4))
        separate['distinct_from'] = [1]
        self.assertEqual(len(self.m.remaining([separate])), 1)
        self.assertEqual(self.m.remaining([self.target(identity=2)]), [])

    def test_revisit_shift_is_uncertain_not_a_second_apple(self):
        self.m.visited = [self.target()]
        duplicate = self.target(identity=7, xy=(.88, 0))
        self.assertEqual(self.m.remaining([duplicate]), [])
        markers = self.m.map_targets([self.target(), duplicate])
        self.assertEqual([m['status'] for m in markers], ['visited', 'ambiguous_revisit'])
        self.assertEqual(len(self.m.visited), 1)
        duplicate['distinct_from'] = [1]
        self.assertEqual(self.m.remaining([duplicate]), [duplicate])

    def test_optional_deadline_is_incomplete_return(self):
        self.nav.limit = 10
        self.m.start(0)
        self.m.command(11, self.scan, 3.5, [], 'running')
        self.assertEqual(self.m.state, 'INCOMPLETE')
        self.assertEqual(self.m.reason, 'explicit_time_limit')

    def test_distant_fresh_observations_do_not_count(self):
        self.m.state, self.m.since = 'VERIFY', 1
        self.m.active = self.target(1)
        for now in (2, 3, 4):
            target = self.target(now)
            target['observed_distance'] = 1.2
            self.m.command(now, self.scan, 3.5, [target], 'running')
        self.assertEqual(self.m.visited, [])

    def test_stale_video_holds_motion(self):
        self.m.start(0)
        self.assertEqual(self.m.command(5, self.scan, 3.5, [], 'stale'), (0, 0))
        self.m.command(36, self.scan, 3.5, [], 'stale')
        self.assertEqual(self.m.reason, 'vision_timeout')

    def test_scan_resumes_existing_frontier(self):
        self.nav.set_mode('EXPLORE', 0)
        goal = self.nav.grid.cell(3, 2)
        self.nav.goal = goal
        self.m.begin_scan(1)
        self.m.scan_angle = 2*math.pi
        self.m.command(5, self.scan, 3.5, [], 'running')
        self.assertEqual(self.m.state, 'SEARCH')
        self.assertEqual(self.nav.goal, goal)

    def test_same_place_does_not_trigger_another_sweep(self):
        self.m.scanned = [[0., 0.]]
        self.m.begin_scan(1)
        self.assertEqual(self.m.state, 'SEARCH')
        self.assertEqual(self.m.scan_count, 0)

    def test_distance_alone_does_not_interrupt_route(self):
        from unittest.mock import Mock
        self.m.state = 'SEARCH'
        self.m.scanned = [[0., 0.]]
        self.nav.pose[:] = [3., 0., 0.]
        self.nav.mode = 'EXPLORE'
        self.nav.command = Mock(return_value=(.12, 0.))
        self.assertEqual(self.m.command(1, self.scan, 3.5, [], 'running'), (.12, 0.))
        self.assertEqual(self.m.state, 'SEARCH')

    def test_exhaustion_with_one_visit_retries_instead_of_returning(self):
        from unittest.mock import Mock
        self.m.state = 'SEARCH'
        self.m.visited = [self.target()]
        self.m.exhaustions = 4
        self.m.attempts = {2: 3}
        self.m.coverage_point = Mock(return_value=None)
        self.nav.mode, self.nav.reason = 'RETURN', 'no_reachable_frontiers'
        self.nav.command = Mock(return_value=(0., 0.))
        self.m.command(10, self.scan, 3.5, [], 'running')
        self.assertEqual(self.m.state, 'SCAN')
        self.assertEqual(len(self.m.visited), 1)
        self.assertEqual(self.m.attempts, {})

    def test_failed_return_recovers_and_completes_only_at_home(self):
        from unittest.mock import Mock
        self.m.visited = [self.target(), self.target(identity=2)]
        self.m.return_home(0, 'two_targets_visited')
        self.nav.pose[0] = 2.
        self.nav.command = Mock(return_value=(0., 0.))
        self.nav.mode, self.nav.reason = 'STOPPED', 'path_unavailable'
        self.m.command(21, self.scan, 3.5, [], 'running')
        self.assertEqual(self.m.state, 'RECOVER_RETURN')
        self.assertEqual(self.m.command(22, [.1]*360, 3.5, [], 'running'), (0., 0.))
        self.assertEqual(self.m.command(23, self.scan, 3.5, [], 'running'), (0., .25))
        self.m.command(29, self.scan, 3.5, [], 'running')
        self.assertEqual(self.m.state, 'RETURN')
        self.assertEqual(self.nav.mode, 'RETURN')
        self.nav.mode = 'HOME'
        self.m.command(30, self.scan, 3.5, [], 'running')
        self.assertEqual(self.m.state, 'SUCCEEDED')


if __name__ == '__main__':
    unittest.main()
