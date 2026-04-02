import argparse
import json
import socket
import time

from air_hockey_planner import AirHockeyPlanner, ExternalPuckEstimate, PlannerConfig, PuckMeasurement, TableBounds


def clamp(value, low, high):
    return max(low, min(high, value))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Bridge estimator output into planner targets and forward to motor controller."
    )
    parser.add_argument("--listen-host", default="0.0.0.0", help="UDP host for estimator packets.")
    parser.add_argument("--listen-port", type=int, default=5006, help="UDP port for estimator packets.")
    parser.add_argument("--motor-host", default="127.0.0.1", help="UDP host for motor controller target packets.")
    parser.add_argument("--motor-port", type=int, default=5007, help="UDP port for motor controller target packets.")
    parser.add_argument("--output-stdout", action="store_true", help="Print outgoing motor packets as JSON.")
    parser.add_argument("--table-length-m", type=float, default=2.0, help="Planner table X range max in meters.")
    parser.add_argument("--table-width-m", type=float, default=0.9, help="Planner table width in meters (Y spans +/- width/2).")
    parser.add_argument("--home-x-m", type=float, default=0.20, help="Planner home X target in meters.")
    parser.add_argument("--defend-x-m", type=float, default=0.18, help="Planner defend X target in meters.")
    parser.add_argument("--goal-x-m", type=float, default=0.12, help="Planner goal/impact line X in meters.")
    parser.add_argument(
        "--prediction-source",
        choices=["estimator", "planner"],
        default="estimator",
        help="Use estimator prediction directly or let planner recompute prediction.",
    )
    return parser.parse_args()


def build_planner_from_args(args):
    half_w = args.table_width_m / 2.0
    config = PlannerConfig(
        table=TableBounds(min_x=0.0, max_x=args.table_length_m, min_y=-half_w, max_y=half_w),
        goal_x=args.goal_x_m,
        defend_x=args.defend_x_m,
        home_x=args.home_x_m,
        home_y=0.0,
        paddle_reach_x_min=0.05,
        paddle_reach_x_max=min(0.75, args.table_length_m),
        csv_log_path=None,
    )
    return AirHockeyPlanner(config=config)


def pixel_to_planner(packet, table_length_m, table_width_m):
    frame_w = int(packet.get("frame_width", 640))
    frame_h = int(packet.get("frame_height", 480))
    x_px = float(packet.get("x", -1.0))
    y_px = float(packet.get("y", -1.0))

    if frame_w <= 1 or frame_h <= 1:
        return None

    x_norm = clamp(x_px / float(frame_w - 1), 0.0, 1.0)
    y_norm = clamp(y_px / float(frame_h - 1), 0.0, 1.0)

    x_m = x_norm * table_length_m
    y_m = (0.5 - y_norm) * table_width_m
    return x_m, y_m


def estimator_packet_to_planner_input(packet, table_length_m, table_width_m):
    detected = bool(packet.get("detected", False))
    if not detected:
        return None

    mapped = pixel_to_planner(packet, table_length_m, table_width_m)
    if mapped is None:
        return None

    x_m, y_m = mapped

    frame_w = int(packet.get("frame_width", 640))
    frame_h = int(packet.get("frame_height", 480))
    if frame_w <= 1 or frame_h <= 1:
        return None

    vx_px_s = float(packet.get("vx_px_s", 0.0))
    vy_px_s = float(packet.get("vy_px_s", 0.0))
    vx_m_s = vx_px_s * (table_length_m / float(frame_w - 1))
    vy_m_s = -vy_px_s * (table_width_m / float(frame_h - 1))

    intercept_valid = bool(packet.get("intercept_valid", False))
    intercept_time = float(packet.get("time_to_intercept_s", -1.0))
    intercept_time = intercept_time if intercept_time >= 0.0 else None

    intercept_y_px = float(packet.get("intercept_y", -1.0))
    intercept_y_m = None
    if intercept_valid and intercept_y_px >= 0.0:
        y_norm = clamp(intercept_y_px / float(frame_h - 1), 0.0, 1.0)
        intercept_y_m = (0.5 - y_norm) * table_width_m

    timestamp = float(packet.get("source_timestamp", packet.get("timestamp", time.time())))
    return ExternalPuckEstimate(
        timestamp=timestamp,
        x=x_m,
        y=y_m,
        vx=vx_m_s,
        vy=vy_m_s,
        confidence=1.0,
        valid_sample_count=1,
        intercept_valid=intercept_valid,
        intercept_time=intercept_time,
        intercept_y=intercept_y_m,
        prediction_confidence=0.8 if intercept_valid else 0.0,
        prediction_reason="External estimator packet.",
    )


def estimator_packet_to_measurement(packet, table_length_m, table_width_m):
    detected = bool(packet.get("detected", False))
    if not detected:
        return None

    mapped = pixel_to_planner(packet, table_length_m, table_width_m)
    if mapped is None:
        return None

    x_m, y_m = mapped
    timestamp = float(packet.get("source_timestamp", packet.get("timestamp", time.time())))
    return PuckMeasurement(timestamp=timestamp, x=x_m, y=y_m, confidence=1.0)


def build_motor_packet(output, source_packet):
    return {
        "timestamp": time.time(),
        "source_timestamp": float(source_packet.get("source_timestamp", source_packet.get("timestamp", 0.0))),
        "command_type": "paddle_target",
        "target_x_m": output.target_x,
        "target_y_m": output.target_y,
        "mode": output.mode.value,
        "reason": output.reason,
        "stale_data": output.stale_data,
        "clamped": output.clamped,
        "safety_limited": output.safety_limited,
        "estimated_puck_vx_m_s": output.estimated_puck_vx,
        "estimated_puck_vy_m_s": output.estimated_puck_vy,
        "debug_summary": output.debug_summary,
    }


def main():
    args = parse_args()
    planner = build_planner_from_args(args)

    input_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    input_socket.bind((args.listen_host, args.listen_port))
    input_socket.settimeout(1.0)

    output_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    motor_target = (args.motor_host, args.motor_port)

    print(
        f"Bridge listening for estimator packets on {args.listen_host}:{args.listen_port}; "
        f"forwarding motor targets to {args.motor_host}:{args.motor_port}"
    )
    print(f"Prediction source mode: {args.prediction_source}")

    try:
        while True:
            try:
                raw, _addr = input_socket.recvfrom(65535)
            except socket.timeout:
                continue

            try:
                estimator_packet = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                continue

            cycle_time = float(estimator_packet.get("source_timestamp", time.time()))
            if args.prediction_source == "estimator":
                planner_input = estimator_packet_to_planner_input(
                    estimator_packet,
                    table_length_m=args.table_length_m,
                    table_width_m=args.table_width_m,
                )
                output = planner.update_from_estimator(estimate=planner_input, now=cycle_time)
            else:
                measurement = estimator_packet_to_measurement(
                    estimator_packet,
                    table_length_m=args.table_length_m,
                    table_width_m=args.table_width_m,
                )
                output = planner.update(measurement=measurement, now=cycle_time)

            motor_packet = build_motor_packet(output, estimator_packet)
            payload = json.dumps(motor_packet).encode("utf-8")
            output_socket.sendto(payload, motor_target)

            if args.output_stdout:
                print(json.dumps(motor_packet), flush=True)
    finally:
        planner.close()
        input_socket.close()
        output_socket.close()


if __name__ == "__main__":
    main()
