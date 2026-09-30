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
    def test_astar_cost_matches_dijkstra_reference(self):
        import networkx as nx
        rng = np.random.default_rng(73)
        for _ in range(8):
            safe = rng.random((12, 12)) > .22
            safe[1, 1] = safe[10, 10] = True
            graph = nx.Graph()
            graph.add_nodes_from(map(tuple, np.argwhere(safe)))
            for r, c in list(graph.nodes):
                for dr, dc in ((1, 0), (0, 1), (1, 1), (1, -1)):
                    other = (r+dr, c+dc)
                    if other not in graph:
                        continue
                    if dr and dc and not (safe[r+dr, c] and safe[r, c+dc]):
                        continue
                    graph.add_edge((r, c), other, weight=math.hypot(dr, dc))
            path = nav.astar(safe, (1, 1), (10, 10))
            try:
                best = nx.dijkstra_path_length(graph, (1, 1), (10, 10))
            except nx.NetworkXNoPath:
                self.assertEqual(path, [])
            else:
                self.assertAlmostEqual(sum(math.dist(a, b) for a, b in zip(path, path[1:])), best)

    def test_frontier_selection_accounts_for_wall_detour(self):
        robot = nav.Navigator()
        robot.grid = nav.GridMap(size=30)
        robot.grid.safe[:] = True
        robot.grid.safe[:25, 11] = False
        start, nearby, reachable = (10, 10), (10, 12), (10, 6)
        robot.pose[:2] = robot.grid.xy(start)
        goal, path = robot.choose_frontier(start, [nearby, reachable])
        self.assertEqual(goal, reachable)
        self.assertEqual(path[-1], reachable)

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

    def test_gyro_heading_is_not_biased_by_wheel_slip(self):
        pose = nav.PoseEstimator()
        pose.update((0, 0), .9, .1)
        for i in range(1, 201):
            # Wheels imply 1 rad/s, but measured body yaw is only .9 rad/s.
            wheel = i*.1*.16/(2*.033)
            pose.update((-wheel, wheel), .9, .1)
        self.assertAlmostEqual(nav.wrap(pose.pose[2]-18.), 0., places=10)
        np.testing.assert_allclose(pose.pose[:2], 0)

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

    def test_return_tries_reachable_home_cell_when_nearest_is_isolated(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = False
        # Nearest cell (0.04, 0.04) is safe but isolated. A parallel corridor
        # reaches another cell inside the SAME 16 cm home tolerance.
        home = robot.grid.cell(0, 0)
        robot.grid.safe[home] = True
        for col in range(home[1], home[1]+12):
            robot.grid.safe[home[0]-2, col] = True
        start = (home[0]-2, home[1]+11)
        robot.plan_home(start)
        self.assertTrue(robot.path)
        self.assertNotEqual(robot.goal, home)
        self.assertLess(np.linalg.norm(robot.grid.xy(robot.goal)), .16)

    def test_return_rejects_outside_radius_and_discards_stale_path(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = False
        cell = robot.grid.cell(.20, .04)
        robot.grid.safe[cell] = True
        robot.path, robot.goal = [cell], cell
        robot.plan_home(cell)
        self.assertEqual(robot.path, [])
        self.assertIsNone(robot.goal)
        self.assertEqual(robot.return_plan_failure, 'home_region_blocked')

    def test_return_does_not_cross_disconnected_map(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = False
        start = robot.grid.cell(2, 0)
        robot.grid.safe[start] = True
        robot.grid.safe[robot.grid.cell(0, 0)] = True
        robot.plan_home(start)
        self.assertEqual(robot.path, [])
        self.assertEqual(robot.return_plan_failure, 'home_region_disconnected')

    def test_blocked_home_uses_safe_recorded_waypoint_without_claiming_arrival(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = True
        r, c = robot.grid.cell(0, 0)
        robot.grid.safe[r-3:r+4, c-3:c+4] = False
        robot.pose[:] = [3, 0, 0]
        robot.trace = [[0, 0], [.8, 0], [1.5, 0], [3, 0]]
        robot.set_mode('RETURN', 0)
        robot.command(1, [3.5]*360, 3.5)
        self.assertEqual(robot.mode, 'RETURN')
        self.assertEqual(robot.return_route_kind, 'recorded_waypoint')
        self.assertLess(np.linalg.norm(robot.grid.xy(robot.goal)), 1)
        self.assertTrue(all(robot.grid.safe[cell] for cell in robot.path))

    def test_historical_route_does_not_override_new_obstacle(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = False
        robot.pose[:] = [3, 0, 0]
        start = robot.grid.cell(3, 0)
        robot.grid.safe[start] = True
        robot.grid.safe[robot.grid.cell(.8, 0)] = True
        robot.trace = [[0, 0], [.8, 0], [3, 0]]
        robot.plan_home(start)
        self.assertEqual(robot.path, [])
        self.assertIsNone(robot.return_route_kind)


if __name__ == "__main__":
    unittest.main()
