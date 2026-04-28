import argparse
import json
import math
import socket
import time

import cv2
import numpy as np


def clamp(value, low, high):
    return max(low, min(high, value))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Render a top-down air hockey table visualization from planner motor packets."
    )
    parser.add_argument("--listen-host", default="127.0.0.1", help="UDP host to bind.")
    parser.add_argument("--listen-port", type=int, default=5007, help="UDP port for planner packets.")
    parser.add_argument("--window-name", default="Air Hockey Visualizer", help="OpenCV window title.")
    parser.add_argument("--canvas-width", type=int, default=1200, help="Visualizer canvas width.")
    parser.add_argument("--canvas-height", type=int, default=720, help="Visualizer canvas height.")
    parser.add_argument(
        "--paddle-max-speed",
        type=float,
        default=1.3,
        help="Simulated paddle max speed in meters/second.",
    )
    parser.add_argument(
        "--paddle-radius-m",
        type=float,
        default=0.055,
        help="Rendered virtual paddle radius in meters.",
    )
    return parser.parse_args()


def table_to_canvas(x_m, y_m, origin_x, origin_y, table_w_px, table_h_px, table_length_m, table_width_m):
    x_norm = 0.0 if table_length_m <= 1e-6 else clamp(x_m / table_length_m, 0.0, 1.0)
    y_norm = 0.5 if table_width_m <= 1e-6 else clamp((0.5 - (y_m / table_width_m)), 0.0, 1.0)
    x_px = int(origin_x + x_norm * table_w_px)
    y_px = int(origin_y + y_norm * table_h_px)
    return x_px, y_px


def draw_label(canvas, text, origin, color, scale=0.65, thickness=2):
    cv2.putText(
        canvas,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def main():
    args = parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen_host, args.listen_port))
    sock.settimeout(0.01)

    cv2.namedWindow(args.window_name)

    latest_packet = None
    packet_received_at = None
    table_length_m = 2.0
    table_width_m = 0.9
    paddle_x = 0.20
    paddle_y = 0.0
    paddle_vx = 0.0
    paddle_vy = 0.0
    last_render_time = time.time()

    print(f"Visualizer listening on {args.listen_host}:{args.listen_port}")
    print("Press Esc to exit.")

    try:
        while True:
            while True:
                try:
                    raw, _addr = sock.recvfrom(65535)
                except socket.timeout:
                    break

                try:
                    latest_packet = json.loads(raw.decode("utf-8"))
                    packet_received_at = time.time()
                    table_length_m = float(latest_packet.get("table_length_m", table_length_m))
                    table_width_m = float(latest_packet.get("table_width_m", table_width_m))
                except json.JSONDecodeError:
                    continue

            now = time.time()
            dt = max(now - last_render_time, 1e-3)
            last_render_time = now

            target_x = float(latest_packet.get("target_x_m", paddle_x)) if latest_packet else paddle_x
            target_y = float(latest_packet.get("target_y_m", paddle_y)) if latest_packet else paddle_y
            dx = target_x - paddle_x
            dy = target_y - paddle_y
            distance = math.hypot(dx, dy)
            max_step = args.paddle_max_speed * dt
            if distance <= max_step or distance <= 1e-9:
                new_x = target_x
                new_y = target_y
            else:
                scale = max_step / distance
                new_x = paddle_x + dx * scale
                new_y = paddle_y + dy * scale
            paddle_vx = (new_x - paddle_x) / dt
            paddle_vy = (new_y - paddle_y) / dt
            paddle_x = new_x
            paddle_y = new_y

            canvas = np.full((args.canvas_height, args.canvas_width, 3), (18, 28, 34), dtype=np.uint8)
            cv2.rectangle(canvas, (0, 0), (args.canvas_width - 1, args.canvas_height - 1), (42, 64, 76), 8)

            margin_x = 70
            margin_y = 80
            sidebar_w = 300
            table_origin_x = margin_x
            table_origin_y = margin_y
            table_w_px = args.canvas_width - (2 * margin_x) - sidebar_w
            table_h_px = args.canvas_height - (2 * margin_y)

            table_color = (199, 228, 233)
            rail_color = (84, 120, 130)
            line_color = (132, 168, 175)
            accent_color = (62, 203, 184)
            puck_color = (20, 20, 20)
            target_color = (126, 152, 255)
            paddle_color = (255, 183, 77)
            goal_color = (96, 120, 255)

            cv2.rectangle(
                canvas,
                (table_origin_x, table_origin_y),
                (table_origin_x + table_w_px, table_origin_y + table_h_px),
                table_color,
                -1,
            )
            cv2.rectangle(
                canvas,
                (table_origin_x, table_origin_y),
                (table_origin_x + table_w_px, table_origin_y + table_h_px),
                rail_color,
                8,
            )

            center_x = table_origin_x + table_w_px // 2
            center_y = table_origin_y + table_h_px // 2
            cv2.line(canvas, (center_x, table_origin_y), (center_x, table_origin_y + table_h_px), line_color, 2)
            cv2.line(canvas, (table_origin_x, center_y), (table_origin_x + table_w_px, center_y), line_color, 2)
            cv2.circle(canvas, (center_x, center_y), max(24, table_h_px // 6), line_color, 2)

            packet_age = None if packet_received_at is None else now - packet_received_at
            has_packet = latest_packet is not None
            home_x_m = float(latest_packet.get("home_x_m", 0.20)) if has_packet else 0.20
            defend_x_m = float(latest_packet.get("defend_x_m", 0.18)) if has_packet else 0.18
            goal_x_m = float(latest_packet.get("goal_x_m", 0.12)) if has_packet else 0.12

            for x_m, label, color in (
                (goal_x_m, "Goal Line", goal_color),
                (defend_x_m, "Defend", (255, 156, 115)),
                (home_x_m, "Home", accent_color),
            ):
                x_px, _ = table_to_canvas(
                    x_m,
                    0.0,
                    table_origin_x,
                    table_origin_y,
                    table_w_px,
                    table_h_px,
                    table_length_m,
                    table_width_m,
                )
                cv2.line(canvas, (x_px, table_origin_y + 10), (x_px, table_origin_y + table_h_px - 10), color, 2)
                draw_label(canvas, label, (x_px + 8, table_origin_y + 28), color, scale=0.48, thickness=1)

            if has_packet and bool(latest_packet.get("puck_detected", False)):
                puck_x = float(latest_packet.get("puck_x_m", 0.0))
                puck_y = float(latest_packet.get("puck_y_m", 0.0))
                puck_pt = table_to_canvas(
                    puck_x,
                    puck_y,
                    table_origin_x,
                    table_origin_y,
                    table_w_px,
                    table_h_px,
                    table_length_m,
                    table_width_m,
                )
                cv2.circle(canvas, puck_pt, 15, (230, 236, 238), -1)
                cv2.circle(canvas, puck_pt, 13, puck_color, -1)

                vx = float(latest_packet.get("estimated_puck_vx_m_s", 0.0))
                vy = float(latest_packet.get("estimated_puck_vy_m_s", 0.0))
                arrow_tip = table_to_canvas(
                    puck_x + (vx * 0.22),
                    puck_y + (vy * 0.22),
                    table_origin_x,
                    table_origin_y,
                    table_w_px,
                    table_h_px,
                    table_length_m,
                    table_width_m,
                )
                if arrow_tip != puck_pt:
                    cv2.arrowedLine(canvas, puck_pt, arrow_tip, puck_color, 3, tipLength=0.22)

                intercept_y = latest_packet.get("predicted_intercept_y_m", None)
                if intercept_y is not None and latest_packet.get("mode") == "DEFENSE":
                    intercept_pt = table_to_canvas(
                        goal_x_m,
                        float(intercept_y),
                        table_origin_x,
                        table_origin_y,
                        table_w_px,
                        table_h_px,
                        table_length_m,
                        table_width_m,
                    )
                    cv2.circle(canvas, intercept_pt, 9, (255, 70, 170), -1)
                    cv2.line(canvas, puck_pt, intercept_pt, (255, 70, 170), 2)

            target_pt = table_to_canvas(
                target_x,
                target_y,
                table_origin_x,
                table_origin_y,
                table_w_px,
                table_h_px,
                table_length_m,
                table_width_m,
            )
            cv2.drawMarker(
                canvas,
                target_pt,
                target_color,
                markerType=cv2.MARKER_CROSS,
                markerSize=24,
                thickness=2,
            )

            paddle_pt = table_to_canvas(
                paddle_x,
                paddle_y,
                table_origin_x,
                table_origin_y,
                table_w_px,
                table_h_px,
                table_length_m,
                table_width_m,
            )
            paddle_radius_px = max(10, int((args.paddle_radius_m / max(table_width_m, 1e-6)) * table_h_px))
            cv2.circle(canvas, paddle_pt, paddle_radius_px + 3, (30, 36, 44), -1)
            cv2.circle(canvas, paddle_pt, paddle_radius_px, paddle_color, -1)
            cv2.circle(canvas, paddle_pt, max(4, paddle_radius_px // 3), (255, 244, 220), -1)

            sidebar_x = table_origin_x + table_w_px + 40
            draw_label(canvas, "Virtual Paddle Demo", (sidebar_x, 90), (240, 248, 250), scale=0.85, thickness=2)

            mode_text = latest_packet.get("mode", "WAITING") if has_packet else "WAITING"
            draw_label(canvas, f"Mode: {mode_text}", (sidebar_x, 145), accent_color, scale=0.72, thickness=2)

            status_text = "No planner packets yet."
            if packet_age is not None:
                if packet_age < 0.30:
                    status_text = f"Live packets ({packet_age:.2f}s old)"
                else:
                    status_text = f"Packets stale ({packet_age:.2f}s old)"
            draw_label(canvas, status_text, (sidebar_x, 182), (215, 226, 229), scale=0.56, thickness=1)

            lines = []
            if has_packet:
                lines = [
                    f"Target x/y: {target_x:.3f}, {target_y:.3f}",
                    f"Paddle x/y: {paddle_x:.3f}, {paddle_y:.3f}",
                    f"Paddle v: {paddle_vx:.2f}, {paddle_vy:.2f} m/s",
                    f"Puck v: {float(latest_packet.get('estimated_puck_vx_m_s', 0.0)):.2f}, "
                    f"{float(latest_packet.get('estimated_puck_vy_m_s', 0.0)):.2f} m/s",
                    f"Intercept t: {latest_packet.get('predicted_intercept_time_s', 'n/a')}",
                    f"Flags: stale={bool(latest_packet.get('stale_data', False))} "
                    f"clamped={bool(latest_packet.get('clamped', False))} "
                    f"limited={bool(latest_packet.get('safety_limited', False))}",
                ]
            else:
                lines = [
                    "Waiting for planner data on UDP.",
                    f"Host/Port: {args.listen_host}:{args.listen_port}",
                ]

            y_cursor = 235
            for line in lines:
                draw_label(canvas, line, (sidebar_x, y_cursor), (230, 237, 239), scale=0.52, thickness=1)
                y_cursor += 34

            draw_label(
                canvas,
                "Esc to close",
                (sidebar_x, args.canvas_height - 35),
                (160, 187, 194),
                scale=0.5,
                thickness=1,
            )

            cv2.imshow(args.window_name, canvas)
            if cv2.waitKey(1) == 27:
                break
    finally:
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
