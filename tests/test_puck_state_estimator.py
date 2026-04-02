import unittest

from puck_state_estimator import estimate_velocity, predict_intercept


class TestPuckStateEstimator(unittest.TestCase):
    def test_estimate_velocity_with_two_samples(self):
        samples = [
            {"timestamp": 10.0, "x": 100.0, "y": 120.0},
            {"timestamp": 10.5, "x": 90.0, "y": 145.0},
        ]

        vx, vy = estimate_velocity(samples)

        self.assertAlmostEqual(vx, -20.0)
        self.assertAlmostEqual(vy, 50.0)

    def test_estimate_velocity_with_insufficient_samples(self):
        vx, vy = estimate_velocity([{"timestamp": 10.0, "x": 100.0, "y": 120.0}])
        self.assertEqual(vx, 0.0)
        self.assertEqual(vy, 0.0)

    def test_predict_intercept_valid_motion(self):
        valid, intercept_x, intercept_y, tti = predict_intercept(
            x=320.0,
            y=200.0,
            vx=-100.0,
            vy=200.0,
            intercept_y=400.0,
            frame_width=640,
        )

        self.assertTrue(valid)
        self.assertAlmostEqual(tti, 1.0)
        self.assertAlmostEqual(intercept_x, 220.0)
        self.assertEqual(intercept_y, 400.0)

    def test_predict_intercept_invalid_if_moving_away(self):
        valid, intercept_x, intercept_y, tti = predict_intercept(
            x=320.0,
            y=420.0,
            vx=50.0,
            vy=100.0,
            intercept_y=300.0,
            frame_width=640,
        )

        self.assertFalse(valid)
        self.assertEqual(intercept_x, -1.0)
        self.assertEqual(intercept_y, 300.0)
        self.assertEqual(tti, -1.0)

    def test_predict_intercept_clamps_x(self):
        valid, intercept_x, _intercept_y, _tti = predict_intercept(
            x=630.0,
            y=100.0,
            vx=1000.0,
            vy=100.0,
            intercept_y=400.0,
            frame_width=640,
        )

        self.assertTrue(valid)
        self.assertEqual(intercept_x, 639.0)


if __name__ == "__main__":
    unittest.main()
