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
    def test_closed_loop_return_crosses_arrival_radius_and_stops(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = True
        robot.pose[:] = [.205, .02, math.pi]
        robot.set_mode('RETURN', 0)
        for step in range(1, 201):
            v, omega = robot.command(step*.064, [3.5]*360, 3.5)
            robot.pose[0] += v*math.cos(robot.pose[2]+omega*.032)*.064
            robot.pose[1] += v*math.sin(robot.pose[2]+omega*.032)*.064
            robot.pose[2] = nav.wrap(robot.pose[2]+omega*.064)
            if robot.mode == 'HOME':
                break
        self.assertEqual(robot.mode, 'HOME')
        self.assertLess(np.linalg.norm(robot.pose[:2]), .16)
        self.assertEqual(robot.command(20, [3.5]*360, 3.5), (0., 0.))

    def test_home_route_is_not_discarded_before_arrival_tolerance(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = True
        robot.pose[:] = [.205, .02, math.pi]
        robot.set_mode('RETURN', 0)
        robot.command(1, [3.5]*360, 3.5)
        goal = robot.goal
        self.assertIsNotNone(goal)
        self.assertLess(np.linalg.norm(robot.pose[:2]-robot.grid.xy(goal)), .22)
        command = robot.command(1.064, [3.5]*360, 3.5)
        self.assertEqual(robot.goal, goal)
        self.assertGreater(command[0], 0)

    def test_multi_goal_route_matches_best_individual_astar(self):
        safe = np.ones((35, 35), dtype=bool)
        safe[5:30, 17] = False
        start = (18, 5)
        goals = [(18, 20), (5, 10), (30, 31)]
        result = nav.astar_to_any(safe, start, goals)
        self.assertAlmostEqual(nav.path_cost(result), min(nav.path_cost(nav.astar(safe, start, g)) for g in goals))
        self.assertEqual(result[-1], (5, 10))
        self.assertEqual(nav.astar_to_any(safe, start, []), [])

    def test_line_of_sight_cannot_cut_blocked_diagonal_corner(self):
        grid = nav.GridMap(size=20)
        grid.safe[:] = True
        grid.safe[9, 10] = False
        self.assertFalse(grid.line_clear(grid.xy((9, 9)), grid.xy((10, 10))))

    def test_following_uses_farther_visible_waypoint_without_replanning(self):
        from unittest.mock import patch
        robot = nav.Navigator()
        robot.grid.safe[:] = True
        robot.go_to((2, 0), 0)
        robot.path = nav.astar(robot.grid.safe, robot.grid.cell(0, 0), robot.goal)
        robot.last_plan = 0
        with patch.object(nav, 'local_command', return_value=(.12, 0)) as drive:
            with patch.object(nav, 'astar', side_effect=AssertionError('Unnecessary replan')):
                robot.command(3, [3.5]*360, 3.5)
            waypoint = drive.call_args.args[1]
            self.assertGreater(np.linalg.norm(waypoint-robot.pose[:2]), .45)
            self.assertLessEqual(np.linalg.norm(waypoint-robot.pose[:2]), .60)

    def test_far_path_obstacle_forces_replan_before_periodic_refresh(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = True
        robot.go_to((3, 0), 0)
        robot.path = nav.astar(robot.grid.safe, robot.grid.cell(0, 0), robot.goal)
        blocked = robot.path[22]
        robot.grid.safe[blocked] = False
        robot.last_plan = 0
        robot.command(1, [3.5]*360, 3.5)
        self.assertNotIn(blocked, robot.path)
        self.assertTrue(robot.path)

    def test_home_recovery_can_backtrack_away_from_home(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = False
        robot.pose[:] = [2., 0., 0.]
        start = robot.grid.cell(2, 0)
        far = robot.grid.cell(3, 0)
        robot.grid.safe[start[0], start[1]:far[1]+1] = True
        robot.trace = [[0, 0], [3, 0], [2, 0]]
        robot.plan_home(start)
        self.assertEqual(robot.return_route_kind, 'recorded_backtrack')
        self.assertEqual(robot.goal, far)
        self.assertTrue(all(robot.grid.safe[p] for p in robot.path))
        robot.plan_home(start)
        self.assertEqual(robot.goal, far)

    def test_observed_departing_obstacle_clears_but_wall_remains(self):
        grid = nav.GridMap(size=100)
        blocked = [math.inf]*360
        blocked[180] = 1.
        for _ in range(5):
            grid.update(np.zeros(3), blocked, 3.5)
        cell = grid.cell(.97, 0)
        self.assertEqual(grid.odds[cell], 30)
        for _ in range(2):
            grid.update(np.zeros(3), [math.inf]*360, 3.5)
        self.assertGreaterEqual(grid.odds[cell], 4)
        grid.update(np.zeros(3), [math.inf]*360, 3.5)
        self.assertTrue(grid.safe[cell])
        for _ in range(10):
            grid.update(np.zeros(3), blocked, 3.5)
        self.assertFalse(grid.safe[cell])

    def test_unobserved_and_occluded_obstacle_is_not_erased(self):
        grid = nav.GridMap(size=100)
        cell = grid.cell(.97, 0)
        grid.odds[cell] = 30
        for _ in range(10):
            grid.update(np.zeros(3), [math.nan]*360, 3.5)
        self.assertEqual(grid.odds[cell], 30)
        for _ in range(10):
            grid.update(np.zeros(3), [.4]*360, 3.5)
        self.assertEqual(grid.odds[cell], 30)

    def test_temporary_block_keeps_goal_then_resumes_for_all_modes(self):
        for mode in ('EXPLORE', 'TARGET', 'RETURN'):
            with self.subTest(mode=mode):
                robot = nav.Navigator()
                robot.grid.safe[:] = True
                robot.pose[:] = [2., 0., math.pi]
                robot.set_mode(mode, 0)
                robot.target_xy = np.zeros(2)
                robot.goal = robot.grid.cell(0, 0)
                robot.path = nav.astar(robot.grid.safe, robot.grid.cell(2, 0), robot.goal)
                original_goal = robot.goal
                blocked_cell = robot.path[6]
                robot.grid.safe[blocked_cell] = False
                self.assertEqual(robot.command(1, [3.5]*360, 3.5), (0., 0.))
                self.assertEqual(robot.reason, 'waiting_for_obstacle')
                self.assertEqual(robot.goal, original_goal)
                robot.grid.safe[blocked_cell] = True
                command = robot.command(2, [3.5]*360, 3.5)
                self.assertEqual(robot.goal, original_goal)
                self.assertEqual(robot.reason, 'obstacle_cleared_resume')
                self.assertGreater(command[0], 0)

    def test_persistent_block_times_out_and_replans_without_crossing(self):
        robot = nav.Navigator()
        robot.grid.safe[:] = True
        robot.go_to((2, 0), 0)
        robot.path = nav.astar(robot.grid.safe, robot.grid.cell(0, 0), robot.goal)
        blocked_cell = robot.path[6]
        robot.grid.safe[blocked_cell] = False
        robot.command(1, [3.5]*360, 3.5)
        robot.command(4.1, [3.5]*360, 3.5)
        self.assertIsNone(robot.obstacle_wait_since)
        self.assertNotIn(blocked_cell, robot.path)
        self.assertTrue(robot.path)

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
