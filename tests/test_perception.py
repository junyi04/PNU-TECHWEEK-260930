import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'controllers/apple_collector'))
from perception import TargetTracker, ground_position, red_region


class PerceptionTests(unittest.TestCase):
    def test_colors(self):
        for color, accepted in [((0, 0, 220), True), ((0, 220, 0), False),
                                ((200, 0, 180), False), ((0, 140, 255), False)]:
            import cv2
            frame = np.zeros((50, 50, 3), np.uint8)
            cv2.circle(frame, (25, 25), 18, color, -1)
            self.assertEqual(red_region(frame, [5, 5, 45, 45]) is not None, accepted)

    def test_projection_rotates_capture_pose(self):
        a = ground_position([290, 220, 350, 285], 640, 480, 1.0472, [0, 0, 0])
        b = ground_position([290, 220, 350, 285], 640, 480, 1.0472, [2, 3, math.pi/2])
        self.assertIsNotNone(a)
        self.assertAlmostEqual(b[0], 2-a[1])
        self.assertAlmostEqual(b[1], 3+a[0])
        self.assertIsNone(ground_position([290, 100, 350, 200], 640, 480, 1.0472, [0, 0, 0]))

    def test_two_targets_revisit_and_duplicate_frames(self):
        t = TargetTracker()
        ds = [dict(position=[1, 0], confidence=.9), dict(position=[2, 1], confidence=.8)]
        t.update(ds, 1)
        t.update(ds, 1)
        self.assertEqual(t.tracks[0]['hits'], 1)
        t.update(ds, 1.2)
        self.assertEqual(len(t.confirmed), 0)
        t.update(ds, 1.4)
        self.assertEqual(len(t.confirmed), 2)
        t.update([], 10)
        t.update(ds, 11)
        self.assertEqual(len(t.confirmed), 2)

    def test_unconfirmed_expires(self):
        t = TargetTracker()
        t.update([dict(position=[1, 0], confidence=.8)], 1)
        t.update([], 5)
        self.assertEqual(t.tracks, [])

    def test_red_rectangle_rejected(self):
        frame = np.full((50, 50, 3), (0, 0, 220), np.uint8)
        self.assertIsNone(red_region(frame, [5, 5, 45, 45]))


if __name__ == '__main__':
    unittest.main()
