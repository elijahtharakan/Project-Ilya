import unittest

import cv2
import numpy as np

from puck_tracker import build_tracker_packet, detect_puck_from_mask


class TestPuckTracker(unittest.TestCase):
    def test_detect_puck_from_mask_finds_circle(self):
        mask = np.zeros((480, 640), dtype=np.uint8)
        cv2.circle(mask, (320, 240), 20, 255, -1)

        detected, center, radius = detect_puck_from_mask(mask, min_area=100.0)

        self.assertTrue(detected)
        self.assertAlmostEqual(center[0], 320, delta=3)
        self.assertAlmostEqual(center[1], 240, delta=3)
        self.assertGreater(radius, 10.0)

    def test_detect_puck_from_mask_rejects_small_blob(self):
        mask = np.zeros((480, 640), dtype=np.uint8)
        cv2.circle(mask, (100, 100), 4, 255, -1)

        detected, center, radius = detect_puck_from_mask(mask, min_area=500.0)

        self.assertFalse(detected)
        self.assertEqual(center, (-1, -1))
        self.assertEqual(radius, 0.0)

    def test_build_tracker_packet_detected(self):
        packet = build_tracker_packet(
            timestamp=123.45,
            detected=True,
            center=(320, 240),
            radius=18.0,
            width=640,
            height=480,
        )

        self.assertTrue(packet["detected"])
        self.assertEqual(packet["x"], 320)
        self.assertEqual(packet["y"], 240)
        self.assertAlmostEqual(packet["x_norm"], 0.5)
        self.assertAlmostEqual(packet["y_norm"], 0.5)
        self.assertEqual(packet["frame_width"], 640)
        self.assertEqual(packet["frame_height"], 480)

    def test_build_tracker_packet_not_detected(self):
        packet = build_tracker_packet(
            timestamp=123.45,
            detected=False,
            center=(-1, -1),
            radius=0.0,
            width=640,
            height=480,
        )

        self.assertFalse(packet["detected"])
        self.assertEqual(packet["x"], -1)
        self.assertEqual(packet["y"], -1)
        self.assertEqual(packet["x_norm"], -1.0)
        self.assertEqual(packet["y_norm"], -1.0)
        self.assertEqual(packet["radius"], 0.0)


if __name__ == "__main__":
    unittest.main()
