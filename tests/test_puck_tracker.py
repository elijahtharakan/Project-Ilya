import unittest
from unittest import mock

import cv2
import numpy as np

from puck_tracker import build_tracker_packet, detect_puck_from_mask, predict_intercept_pixel, resolve_camera_index


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

    def test_resolve_camera_index_prefers_external_webcam_in_auto_mode(self):
        with mock.patch("puck_tracker._probe_camera_index", side_effect=lambda index: index == 2):
            self.assertEqual(resolve_camera_index("auto"), 2)

    def test_resolve_camera_index_falls_back_to_zero_in_auto_mode(self):
        with mock.patch("puck_tracker._probe_camera_index", side_effect=lambda index: index == 0):
            self.assertEqual(resolve_camera_index("auto"), 0)

    def test_predict_intercept_pixel_valid(self):
        valid, x_pred, y_pred, tti = predict_intercept_pixel(
            center=(320, 200),
            vx=100.0,
            vy=200.0,
            intercept_y=400,
            frame_width=640,
            max_time_s=3.0,
        )

        self.assertTrue(valid)
        self.assertAlmostEqual(tti, 1.0, places=3)
        self.assertAlmostEqual(x_pred, 420.0, places=3)
        self.assertEqual(y_pred, 400.0)

    def test_predict_intercept_pixel_invalid_if_moving_away(self):
        valid, x_pred, _y_pred, tti = predict_intercept_pixel(
            center=(320, 420),
            vx=50.0,
            vy=100.0,
            intercept_y=300,
            frame_width=640,
            max_time_s=3.0,
        )

        self.assertFalse(valid)
        self.assertEqual(x_pred, -1.0)
        self.assertEqual(tti, -1.0)


if __name__ == "__main__":
    unittest.main()
