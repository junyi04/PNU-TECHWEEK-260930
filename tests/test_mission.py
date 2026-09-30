import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'controllers/apple_collector'))
from navigation import Navigator
from mission import Mission


class MissionTests(unittest.TestCase):
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
                self.m.command(now, self.scan, 3.5, [self.target(now, identity, xy)], 'running')
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
        self.assertEqual(len(self.m.remaining([self.target(identity=2, xy=(.43, .4))])), 1)
        self.assertEqual(self.m.remaining([self.target(identity=2)]), [])

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


if __name__ == '__main__':
    unittest.main()
