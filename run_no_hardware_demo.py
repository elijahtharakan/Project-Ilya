import argparse
import subprocess
import sys
import time
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run full no-hardware air hockey pipeline with one command."
    )
    parser.add_argument("--prediction-source", choices=["estimator", "planner"], default="estimator")
    parser.add_argument("--mode", choices=["showcase", "toward", "away", "zigzag", "attack"], default="showcase")
    parser.add_argument("--feed-seconds", type=float, default=12.0)
    parser.add_argument("--stay-alive", action="store_true", help="Keep services running after feeder exits.")
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
    ])

    feeder_cmd = [
        sys.executable,
        str(root / "synthetic_puck_feed.py"),
        "--host",
        "127.0.0.1",
        "--port",
        "5005",
        "--mode",
        args.mode,
        "--seconds",
        str(args.feed_seconds),
    ]

    processes = []

    print("Starting no-hardware pipeline services...")
    try:
        for cmd in commands:
            proc = subprocess.Popen(cmd, cwd=str(root))
            processes.append(proc)
            time.sleep(0.4)

        print("Running synthetic puck feed...")
        subprocess.run(feeder_cmd, cwd=str(root), check=True)

        if args.stay_alive:
            print("Feeder finished. Services still running. Press Ctrl+C to stop.")
            while True:
                time.sleep(0.5)
        else:
            print("Feeder finished. Stopping services...")

    except KeyboardInterrupt:
        print("Interrupted by user. Stopping services...")
    finally:
        terminate_processes(processes)
        print("All processes stopped.")


if __name__ == "__main__":
    main()
