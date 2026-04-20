import json
import socket
import subprocess
import sys
import time
import unittest
from pathlib import Path


class TestNoHardwarePipeline(unittest.TestCase):
    def test_bridge_emits_motor_packet_from_estimator_input(self):
        root = Path(__file__).resolve().parent.parent
        bridge_path = root / "planner_motor_bridge.py"

        in_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        in_sock.bind(("127.0.0.1", 0))
        bridge_input_port = in_sock.getsockname()[1]
        in_sock.close()

        motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        motor_sock.bind(("127.0.0.1", 0))
        motor_port = motor_sock.getsockname()[1]
        motor_sock.settimeout(4.0)

        proc = subprocess.Popen(
            [
                sys.executable,
                str(bridge_path),
                "--listen-host",
                "127.0.0.1",
                "--listen-port",
                str(bridge_input_port),
                "--motor-host",
                "127.0.0.1",
                "--motor-port",
                str(motor_port),
                "--prediction-source",
                "estimator",
            ],
            cwd=str(root),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        try:
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            raw = None
            for _ in range(20):
                if proc.poll() is not None:
                    self.fail("planner_motor_bridge.py exited unexpectedly during test")

                estimator_packet = {
                    "timestamp": time.time(),
                    "source_timestamp": time.time(),
                    "detected": True,
                    "x": 320,
                    "y": 220,
                    "vx_px_s": -200.0,
                    "vy_px_s": 30.0,
                    "intercept_valid": True,
                    "intercept_y": 420.0,
                    "time_to_intercept_s": 0.8,
                    "frame_width": 640,
                    "frame_height": 480,
                }
                sender.sendto(
                    json.dumps(estimator_packet).encode("utf-8"),
                    ("127.0.0.1", bridge_input_port),
                )

                try:
                    raw, _addr = motor_sock.recvfrom(65535)
                    break
                except TimeoutError:
                    continue

            sender.close()
            self.assertIsNotNone(raw, "Did not receive motor packet from bridge")
            output_packet = json.loads(raw.decode("utf-8"))

            self.assertEqual(output_packet.get("command_type"), "paddle_target")
            self.assertIn(output_packet.get("mode"), {"DEFENSE", "ATTACK", "HOME", "SAFE_HOLD"})
            self.assertIsInstance(output_packet.get("target_x_m"), float)
            self.assertIsInstance(output_packet.get("target_y_m"), float)
            self.assertTrue(output_packet.get("puck_detected"))
            self.assertIsInstance(output_packet.get("puck_x_m"), float)
            self.assertIsInstance(output_packet.get("puck_y_m"), float)
            self.assertIn("predicted_intercept_time_s", output_packet)
            self.assertIn("predicted_intercept_y_m", output_packet)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            motor_sock.close()


if __name__ == "__main__":
    unittest.main()
