import json
import os
import socket
import subprocess
import sys
import unittest
from pathlib import Path


@unittest.skipUnless(os.getenv("RUN_CAMERA_TESTS") == "1", "Set RUN_CAMERA_TESTS=1 to run webcam tests.")
class TestCameraPipeline(unittest.TestCase):
    def test_tracker_emits_packets_from_webcam(self):
        root = Path(__file__).resolve().parent.parent
        tracker_path = root / "puck_tracker.py"

        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.bind(("127.0.0.1", 0))
        recv_port = recv_sock.getsockname()[1]
        recv_sock.settimeout(10.0)

        proc = subprocess.Popen(
            [
                sys.executable,
                str(tracker_path),
                "--camera",
                "0",
                "--width",
                "640",
                "--height",
                "480",
                "--udp-host",
                "127.0.0.1",
                "--udp-port",
                str(recv_port),
                "--no-gui",
                "--max-frames",
                "40",
            ],
            cwd=str(root),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        try:
            raw, _addr = recv_sock.recvfrom(65535)
            packet = json.loads(raw.decode("utf-8"))

            self.assertIn("timestamp", packet)
            self.assertIn("detected", packet)
            self.assertIn("x", packet)
            self.assertIn("y", packet)
            self.assertEqual(packet.get("frame_width"), 640)
            self.assertEqual(packet.get("frame_height"), 480)
            self.assertIsInstance(packet.get("detected"), bool)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            recv_sock.close()


if __name__ == "__main__":
    unittest.main()
