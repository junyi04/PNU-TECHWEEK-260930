"""Safety/sequence regressions; run: python -m unittest discover -s tests -v."""
import importlib.util
import math
from pathlib import Path
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "controllers/apple_collector/apple_collector.py"
spec = importlib.util.spec_from_file_location("stage1", SOURCE)
stage1 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stage1)


class SafetyTests(unittest.TestCase):
    def scan(self, index, distance):
        ranges = [math.inf] * 360
        ranges[index] = distance
        return stage1.scan_geometry(ranges, 3.5)

    def test_front_and_rear_orientation(self):
        self.assertAlmostEqual(self.scan(180, 0.30)["front"], 0.27)
        self.assertAlmostEqual(self.scan(0, 0.30)["rear"], 0.33)

    def test_front_obstacle_stops_forward_but_allows_retreat(self):
        scan = self.scan(180, 0.25)
        self.assertEqual(stage1.protect_motion(0.066, 0, scan), (0, 0, "FRONT_BLOCKED"))
        self.assertLess(stage1.protect_motion(-0.066, 0, scan)[0], 0)

    def test_rear_obstacle_blocks_reverse(self):
        self.assertEqual(stage1.protect_motion(-0.066, 0, self.scan(0, 0.20))[2],
                         "REAR_BLOCKED")

    def test_side_obstacle_blocks_swept_rotation(self):
        self.assertEqual(stage1.protect_motion(0, 0.55, self.scan(90, 0.13))[2],
                         "ROTATION_BLOCKED")

    def test_invalid_or_empty_scan_fails_closed(self):
        for values in ([], [math.inf] * 360, [math.nan] * 360, [-math.inf] * 360,
                       [0.0] * 360):
            self.assertIsNone(stage1.scan_geometry(values, 3.5))
        self.assertEqual(stage1.protect_motion(0.066, 0, None), (0, 0, "SENSOR_NOT_READY"))

    def test_mixed_invalid_ray_fails_closed(self):
        values = [1.0] * 360
        values[180] = math.nan
        self.assertIsNone(stage1.scan_geometry(values, 3.5))

    def test_slowdown_before_stop(self):
        speed, _, state = stage1.protect_motion(0.066, 0, self.scan(180, 0.40))
        self.assertGreater(speed, 0)
        self.assertLess(speed, 0.066)
        self.assertEqual(state, "SLOWING")

    def test_wheel_limit_preserves_curvature(self):
        left, right = stage1.wheel_speeds(1.0, 0.55, 6.67)
        self.assertLessEqual(max(abs(left), abs(right)), 6.67)
        self.assertGreater(right, left)


class SequenceTests(unittest.TestCase):
    clear = {"front": 3.5, "rear": 3.5, "around": 1.0}

    def test_sequence_with_encoder_feedback_and_obstacle_stop(self):
        demo = stage1.Checkout(0, (0, 0))
        self.assertGreater(demo.command(0.1, (0, 0), self.clear)[0], 0)
        straight = 0.21 / stage1.WHEEL_RADIUS
        demo.command(4.0, (straight, straight), self.clear)
        self.assertEqual(demo.phase, "STOP_1")
        demo.command(4.7, (straight, straight), self.clear)
        self.assertGreater(demo.command(4.8, (straight, straight), self.clear)[1], 0)
        turn = (math.pi / 4 + 0.02) * stage1.AXLE_LENGTH / (2 * stage1.WHEEL_RADIUS)
        turned = (straight - turn, straight + turn)
        demo.command(6.3, turned, self.clear)
        demo.command(7.0, turned, self.clear)
        self.assertEqual(demo.phase, "TURN_RIGHT")
        self.assertLess(demo.command(7.1, turned, self.clear)[1], 0)
        demo.command(8.6, (straight, straight), self.clear)
        demo.command(9.3, (straight, straight), self.clear)
        self.assertEqual(demo.phase, "APPROACH")
        blocked = {**self.clear, "front": 0.25}
        self.assertEqual(demo.command(10, (straight, straight), blocked), (0, 0))
        self.assertEqual(demo.result, "obstacle_stop")
        self.assertEqual(demo.command(11, (straight, straight), self.clear), (0, 0))

    def test_forward_stall_is_bounded(self):
        demo = stage1.Checkout(0, (0, 0))
        self.assertEqual(demo.command(9, (0, 0), self.clear), (0, 0))
        self.assertEqual(demo.result, "forward_timeout")

    def test_no_obstacle_is_not_reported_as_a_pass(self):
        demo = stage1.Checkout(0, (0, 0))
        demo.index = demo.PHASES.index("APPROACH")
        demo.command(46, (0, 0), self.clear)
        self.assertEqual(demo.result, "travel_limit_no_obstacle")


if __name__ == "__main__":
    unittest.main()
