import argparse
import json
import math
import socket
import time
from collections import deque


def parse_args():
    parser = argparse.ArgumentParser(
        description="Consumes puck coordinates and outputs velocity + intercept prediction."
    )
    parser.add_argument("--listen-host", default="0.0.0.0", help="UDP host to bind.")
    parser.add_argument("--listen-port", type=int, default=5005, help="UDP port to bind.")
    parser.add_argument(
        "--history-size",
        type=int,
        default=6,
        help="Number of recent detected samples used for velocity estimate.",
    )
    parser.add_argument(
        "--intercept-y",
        type=float,
        default=420.0,
        help="Y line (in camera pixels) where robot expects to intercept puck.",
    )
    parser.add_argument(
        "--frame-width",
        type=int,
        default=640,
        help="Used to clamp predicted intercept X.",
    )
    parser.add_argument(
        "--output-stdout",
        action="store_true",
        help="Print enriched state as JSON packets.",
    )
    parser.add_argument(
        "--output-udp-host",
        default="",
        help="Optional UDP host to send enriched packets to controller.",
    )
    parser.add_argument(
        "--output-udp-port",
        type=int,
        default=5006,
        help="Optional UDP port for enriched packets.",
    )
    return parser.parse_args()


def estimate_velocity(samples):
    """Estimate velocity from oldest/newest sample for simple robust smoothing."""
    if len(samples) < 2:
        return 0.0, 0.0

    first = samples[0]
    last = samples[-1]
    dt = last["timestamp"] - first["timestamp"]
    if dt <= 0:
        return 0.0, 0.0

    vx = (last["x"] - first["x"]) / dt
    vy = (last["y"] - first["y"]) / dt
    return vx, vy


def predict_intercept(x, y, vx, vy, intercept_y, frame_width):
    """Predict where the puck crosses intercept_y if moving toward that line."""
    if abs(vy) < 1e-6:
        return False, -1.0, intercept_y, -1.0

    time_to_intercept = (intercept_y - y) / vy
    if time_to_intercept <= 0:
        return False, -1.0, intercept_y, -1.0

    intercept_x = x + (vx * time_to_intercept)
    intercept_x = max(0.0, min(float(frame_width - 1), intercept_x))
    return True, intercept_x, intercept_y, time_to_intercept


def main():
    args = parse_args()

    input_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    input_socket.bind((args.listen_host, args.listen_port))
    input_socket.settimeout(1.0)

    output_socket = None
    output_target = None
    if args.output_udp_host:
        output_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        output_target = (args.output_udp_host, args.output_udp_port)

    samples = deque(maxlen=max(2, args.history_size))

    print(
        f"Estimator listening on {args.listen_host}:{args.listen_port}; "
        f"intercept_y={args.intercept_y}"
    )
    if output_target is not None:
        print(f"Estimator forwarding packets to {output_target[0]}:{output_target[1]}")

    while True:
        try:
            raw, _addr = input_socket.recvfrom(65535)
        except socket.timeout:
            continue

        now = time.time()
        try:
            packet = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            continue

        detected = bool(packet.get("detected", False))
        x = float(packet.get("x", -1))
        y = float(packet.get("y", -1))
        timestamp = float(packet.get("timestamp", now))

        if detected and x >= 0 and y >= 0:
            samples.append({"timestamp": timestamp, "x": x, "y": y})
        else:
            samples.clear()

        vx, vy = estimate_velocity(samples)
        speed = math.hypot(vx, vy)

        valid, intercept_x, intercept_y, tti = predict_intercept(
            x=x,
            y=y,
            vx=vx,
            vy=vy,
            intercept_y=args.intercept_y,
            frame_width=args.frame_width,
        )

        enriched = {
            "timestamp": now,
            "source_timestamp": timestamp,
            "detected": detected,
            "x": x,
            "y": y,
            "vx_px_s": vx,
            "vy_px_s": vy,
            "speed_px_s": speed,
            "intercept_valid": bool(valid and detected),
            "intercept_x": float(intercept_x) if (valid and detected) else -1.0,
            "intercept_y": float(intercept_y) if detected else -1.0,
            "time_to_intercept_s": float(tti) if (valid and detected) else -1.0,
        }

        payload = json.dumps(enriched)

        if args.output_stdout:
            print(payload, flush=True)

        if output_socket is not None and output_target is not None:
            output_socket.sendto(payload.encode("utf-8"), output_target)


if __name__ == "__main__":
    main()