import argparse
import subprocess
import sys
import time
from pathlib import Path

#python run_pi_demo.py --visualizer --flip-horizontal


def parse_args():
    parser = argparse.ArgumentParser(description="Run the camera-backed air hockey pipeline on a Raspberry Pi.")
    parser.add_argument(
        "--camera",
        default="auto",
        help="Camera index for puck_tracker.py, or 'auto' to prefer an external webcam (default: auto).",
    )
    parser.add_argument(
        "--capture-backend",
        choices=["opencv", "picamera2"],
        default="opencv",
        help="Camera backend for puck_tracker.py (opencv for USB webcam, picamera2 for Pi camera module).",
    )
    parser.add_argument("--width", type=int, default=640, help="Tracker frame width (default: 640).")
    parser.add_argument("--height", type=int, default=480, help="Tracker frame height (default: 480).")
    parser.add_argument(
        "--flip-horizontal",
        action="store_true",
        help="Flip the camera feed left-to-right before tracking and display.",
    )
    parser.add_argument(
        "--flip-vertical",
        action="store_true",
        help="Flip the camera feed top-to-bottom before tracking and display.",
    )
    parser.add_argument(
        "--prediction-source",
        choices=["estimator", "planner"],
        default="estimator",
        help="Whether the bridge uses estimator prediction directly or planner recomputation.",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Run puck_tracker.py without windows (useful if the Pi is headless).",
    )
    parser.add_argument(
        "--stay-alive",
        action="store_true",
        help="Keep the supporting processes running after the tracker exits.",
    )
    parser.add_argument(
        "--visualizer",
        action="store_true",
        help="Show the top-down table visualizer instead of the mock motor receiver.",
    )
    return parser.parse_args()


def terminate_processes(processes):
    for proc in reversed(processes):
        if proc.poll() is None:
            proc.terminate()
    for proc in reversed(processes):
        if proc.poll() is None:
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()


def main():
    args = parse_args()
    root = Path(__file__).resolve().parent

    commands = []
    if args.visualizer:
        commands.append(
            [
                sys.executable,
                str(root / "table_visualizer.py"),
                "--listen-host",
                "127.0.0.1",
                "--listen-port",
                "5007",
            ]
        )
    else:
        commands.append(
            [
                sys.executable,
                str(root / "mock_motor_controller.py"),
                "--listen-host",
                "127.0.0.1",
                "--listen-port",
                "5007",
            ]
        )

    commands.extend([
        [
            sys.executable,
            str(root / "planner_motor_bridge.py"),
            "--listen-host",
            "127.0.0.1",
            "--listen-port",
            "5006",
            "--motor-host",
            "127.0.0.1",
            "--motor-port",
            "5007",
            "--prediction-source",
            args.prediction_source,
            "--output-stdout",
        ],
        [
            sys.executable,
            str(root / "puck_state_estimator.py"),
            "--listen-host",
            "127.0.0.1",
            "--listen-port",
            "5005",
            "--output-udp-host",
            "127.0.0.1",
            "--output-udp-port",
            "5006",
            "--intercept-x",
            "38",
        ],
        [
            sys.executable,
            str(root / "puck_tracker.py"),
            "--camera",
            str(args.camera),
            "--capture-backend",
            args.capture_backend,
            "--width",
            str(args.width),
            "--height",
            str(args.height),
            "--udp-host",
            "127.0.0.1",
            "--udp-port",
            "5005",
        ],
    ])

    if args.no_gui:
        commands[-1].append("--no-gui")
    if args.flip_horizontal:
        commands[-1].append("--flip-horizontal")
    if args.flip_vertical:
        commands[-1].append("--flip-vertical")

    processes = []
    print("Starting Pi camera pipeline...")

    try:
        for cmd in commands:
            proc = subprocess.Popen(cmd, cwd=str(root))
            processes.append(proc)
            time.sleep(0.5)

        if args.stay_alive:
            print("Pipeline started. Press Ctrl+C to stop.")
            while True:
                time.sleep(0.5)
        else:
            tracker_proc = processes[-1]
            tracker_proc.wait()

    except KeyboardInterrupt:
        print("Interrupted. Stopping pipeline...")
    finally:
        terminate_processes(processes)
        print("All processes stopped.")


if __name__ == "__main__":
    main()
