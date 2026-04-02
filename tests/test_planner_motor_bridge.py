import unittest

from planner_motor_bridge import (
    build_motor_packet,
    estimator_packet_to_measurement,
    estimator_packet_to_planner_input,
    pixel_to_planner,
)


class DummyMode:
    value = "DEFENSE"


class DummyOutput:
    target_x = 0.2
    target_y = -0.1
    mode = DummyMode()
    reason = "test"
    stale_data = False
    clamped = False
    safety_limited = False
    estimated_puck_vx = -0.3
    estimated_puck_vy = 0.1
    debug_summary = "ok"


class TestPlannerMotorBridge(unittest.TestCase):
    def test_pixel_to_planner_center(self):
        packet = {"x": 320, "y": 240, "frame_width": 640, "frame_height": 480}
        x_m, y_m = pixel_to_planner(packet, table_length_m=2.0, table_width_m=0.9)

        self.assertAlmostEqual(x_m, 320 / 639 * 2.0, places=6)
        self.assertAlmostEqual(y_m, (0.5 - (240 / 479)) * 0.9, places=6)

    def test_estimator_packet_to_planner_input_detected_false(self):
        packet = {"detected": False, "x": -1, "y": -1, "frame_width": 640, "frame_height": 480}
        estimate = estimator_packet_to_planner_input(packet, table_length_m=2.0, table_width_m=0.9)
        self.assertIsNone(estimate)

    def test_estimator_packet_to_measurement_detected_true(self):
        packet = {
            "detected": True,
            "x": 200,
            "y": 100,
            "frame_width": 640,
            "frame_height": 480,
            "source_timestamp": 12.5,
        }
        measurement = estimator_packet_to_measurement(packet, table_length_m=2.0, table_width_m=0.9)

        self.assertIsNotNone(measurement)
        self.assertAlmostEqual(measurement.timestamp, 12.5)
        self.assertGreaterEqual(measurement.x, 0.0)
        self.assertLessEqual(measurement.x, 2.0)
        self.assertGreaterEqual(measurement.y, -0.45)
        self.assertLessEqual(measurement.y, 0.45)

    def test_estimator_packet_to_planner_input_detected_true(self):
        packet = {
            "detected": True,
            "x": 200,
            "y": 100,
            "frame_width": 640,
            "frame_height": 480,
            "source_timestamp": 12.5,
            "vx_px_s": -120.0,
            "vy_px_s": 80.0,
            "intercept_valid": True,
            "intercept_y": 420.0,
            "time_to_intercept_s": 0.75,
        }
        estimate = estimator_packet_to_planner_input(packet, table_length_m=2.0, table_width_m=0.9)

        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.timestamp, 12.5)
        self.assertGreaterEqual(estimate.x, 0.0)
        self.assertLessEqual(estimate.x, 2.0)
        self.assertGreaterEqual(estimate.y, -0.45)
        self.assertLessEqual(estimate.y, 0.45)
        self.assertLess(estimate.vx, 0.0)
        self.assertLess(estimate.vy, 0.0)
        self.assertTrue(estimate.intercept_valid)
        self.assertAlmostEqual(estimate.intercept_time, 0.75)

    def test_build_motor_packet(self):
        src = {"timestamp": 1.0, "source_timestamp": 0.9}
        packet = build_motor_packet(DummyOutput(), src)

        self.assertEqual(packet["command_type"], "paddle_target")
        self.assertEqual(packet["mode"], "DEFENSE")
        self.assertEqual(packet["target_x_m"], 0.2)
        self.assertEqual(packet["target_y_m"], -0.1)


if __name__ == "__main__":
    unittest.main()
