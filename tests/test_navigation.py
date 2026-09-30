import importlib.util
import math
from pathlib import Path
import unittest
import numpy as np

path = Path(__file__).resolve().parents[1] / "controllers/apple_collector/navigation.py"
spec = importlib.util.spec_from_file_location("navigation", path)
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)


class NavigationTests(unittest.TestCase):
    def test_astar_detours_and_never_crosses_blocked_corner(self):
        safe = np.ones((12, 12), dtype=bool)
        safe[0:10, 6] = False
        path = nav.astar(safe, (3, 2), (3, 9))
        self.assertEqual(path[0], (3, 2))
        self.assertEqual(path[-1], (3, 9))
        self.assertTrue(any(r >= 10 for r, c in path))
        for a, b in zip(path, path[1:]):
            self.assertTrue(safe[b])
            if a[0] != b[0] and a[1] != b[1]:
                self.assertTrue(safe[a[0], b[1]] and safe[b[0], a[1]])

    def test_no_route_does_not_cut_unknown(self):
        safe = np.zeros((8, 8), dtype=bool)
        safe[1, 1] = safe[6, 6] = True
        self.assertEqual(nav.astar(safe, (1, 1), (6, 6)), [])

    def test_encoder_only_pose_and_gyro_override(self):
        pose = nav.PoseEstimator()
        pose.update((0, 0), None, .064)
        pose.update((1, 1), None, .064)
        self.assertAlmostEqual(pose.pose[0], .033)
        self.assertAlmostEqual(pose.pose[1], 0)
        pose.update((0, 2), None, .064)
        self.assertAlmostEqual(pose.pose[2], .066/.16)

    def test_mapping_front_wall_and_inflation(self):
        grid = nav.GridMap(size=100)
        scan = [math.inf] * 360
        for i in range(150, 211):
            scan[i] = 1.0 / math.cos(math.pi-2*math.pi*i/360)
        grid.update(np.zeros(3), scan, 3.5)
        self.assertGreaterEqual(grid.odds[grid.cell(.97, 0)], 4)
        self.assertFalse(grid.safe[grid.cell(.85, 0)])
        self.assertTrue(grid.safe[grid.cell(.3, 0)])

    def test_local_planner_does_not_drive_into_wall(self):
        scan = [math.inf] * 360
        for i in range(130, 231):
            scan[i] = .18/math.cos(math.pi-2*math.pi*i/360)
        linear, _ = nav.local_command(np.zeros(3), np.array([1., 0.]), scan, 3.5)
        self.assertEqual(linear, 0)

    def test_timeout_requests_return_and_home_is_terminal(self):
        robot = nav.Navigator(explore_seconds=1)
        robot.set_mode("EXPLORE", 0)
        self.assertEqual(robot.command(2, [1]*360, 3.5), (0, 0))
        self.assertEqual(robot.mode, "HOME")


if __name__ == "__main__":
    unittest.main()
