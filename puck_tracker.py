import argparse
import json
import socket
import time

import cv2
import numpy as np

# Global variable to store the latest HSV frame for the mouse callback.
current_hsv_frame = None


def nothing(_x):
    """Callback function for trackbars (does nothing)."""
    return


def pick_color(event, x, y, _flags, _param):
    """Mouse callback to pick color from the frame and auto-tune HSV bounds."""
    global current_hsv_frame

    if event == cv2.EVENT_LBUTTONDOWN and current_hsv_frame is not None:
        pixel = current_hsv_frame[y, x]
        h_val, s_val, v_val = int(pixel[0]), int(pixel[1]), int(pixel[2])

        print(f"Picked HSV: {pixel}")

        # Tolerance around the sampled puck color.
        h_lower = max(0, h_val - 10)
        h_upper = min(179, h_val + 10)
        s_lower = max(0, s_val - 50)
        v_lower = max(0, v_val - 50)

        cv2.setTrackbarPos("L-H", "Trackbars", h_lower)
        cv2.setTrackbarPos("L-S", "Trackbars", s_lower)
        cv2.setTrackbarPos("L-V", "Trackbars", v_lower)
        cv2.setTrackbarPos("U-H", "Trackbars", h_upper)
        cv2.setTrackbarPos("U-S", "Trackbars", 255)
        cv2.setTrackbarPos("U-V", "Trackbars", 255)


def parse_args():
    parser = argparse.ArgumentParser(description="Live puck tracker with coordinate publishing.")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument(
        "--capture-backend",
        choices=["opencv", "picamera2"],
        default="opencv",
        help="Camera backend to use (opencv for USB/webcam, picamera2 for Raspberry Pi camera module).",
    )
    parser.add_argument("--width", type=int, default=640, help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Frame height (default: 480)")
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
        "--min-area",
        type=float,
        default=500.0,
        help="Minimum contour area to accept puck detection (default: 500)",
    )
    parser.add_argument(
        "--send-stdout",
        action="store_true",
        help="Print JSON coordinate packets to stdout for downstream processes.",
    )
    parser.add_argument(
        "--udp-host",
        default="",
        help="If set, send JSON coordinate packets over UDP to this host.",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=5005,
        help="UDP port for coordinate packets (default: 5005).",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Run without display windows (useful for automated tests/headless runs).",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after this many frames (0 means run until interrupted).",
    )
    parser.add_argument(
        "--velocity-arrow-scale",
        type=float,
        default=0.15,
        help="Arrow lookahead in seconds for velocity visualization (default: 0.15).",
    )
    parser.add_argument(
        "--intercept-x-px",
        "--intercept-y-px",
        dest="intercept_x_px",
        type=float,
        default=38.0,
        help="Image X column used for live intercept prediction overlay (default: 38).",
    )
    parser.add_argument(
        "--max-intercept-time",
        type=float,
        default=3.0,
        help="Ignore intercept predictions farther than this many seconds out.",
    )
    return parser.parse_args()


class OpenCVCapture:
    def __init__(self, camera_index):
        self._capture = cv2.VideoCapture(camera_index)

    def isOpened(self):
        return self._capture.isOpened()

    def read(self):
        return self._capture.read()

    def release(self):
        self._capture.release()


class Picamera2Capture:
    def __init__(self, width, height):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "Picamera2 backend selected but picamera2 is not installed. "
                "Install python3-picamera2 on the Pi or use --capture-backend opencv."
            ) from exc

        self._picamera2 = Picamera2()
        config = self._picamera2.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
        self._picamera2.configure(config)
        self._picamera2.start()
        self._opened = True

    def isOpened(self):
        return self._opened

    def read(self):
        frame_rgb = self._picamera2.capture_array()
        if frame_rgb is None:
            return False, None
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        return True, frame_bgr

    def release(self):
        self._picamera2.stop()
        self._opened = False


def create_capture(args):
    if args.capture_backend == "picamera2":
        return Picamera2Capture(args.width, args.height)
    return OpenCVCapture(args.camera)


def detect_puck_from_mask(mask, min_area):
    """Detect puck from a binary mask using largest contour strategy."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = False
    center = (-1, -1)
    radius = 0.0

    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > min_area:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            moments = cv2.moments(c)
            if moments["m00"] != 0:
                center = (
                    int(moments["m10"] / moments["m00"]),
                    int(moments["m01"] / moments["m00"]),
                )
            else:
                center = (int(x), int(y))
            detected = True

    return detected, center, float(radius)


def build_tracker_packet(timestamp, detected, center, radius, width, height):
    """Build wire-format packet from tracker state."""
    return {
        "timestamp": float(timestamp),
        "detected": bool(detected),
        "x": int(center[0]),
        "y": int(center[1]),
        "x_norm": float(center[0] / width) if detected else -1.0,
        "y_norm": float(center[1] / height) if detected else -1.0,
        "radius": float(radius) if detected else 0.0,
        "frame_width": int(width),
        "frame_height": int(height),
    }


def predict_intercept_pixel(center, vx, vy, intercept_x, frame_height, max_time_s):
    """Predict where current motion crosses intercept_x in pixel coordinates."""
    if abs(vx) < 1e-6:
        return False, float(intercept_x), -1.0, -1.0

    tti = (float(intercept_x) - float(center[0])) / float(vx)
    if tti <= 0.0 or tti > max_time_s:
        return False, float(intercept_x), -1.0, -1.0

    y_pred = float(center[1]) + float(vy) * tti
    y_pred = max(0.0, min(float(frame_height - 1), y_pred))
    return True, float(intercept_x), y_pred, float(tti)


def main():
    global current_hsv_frame
    args = parse_args()

    try:
        cap = create_capture(args)
    except RuntimeError as exc:
        print(str(exc))
        return

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    udp_socket = None
    udp_target = None
    if args.udp_host:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_target = (args.udp_host, args.udp_port)
        print(f"UDP output enabled: {udp_target[0]}:{udp_target[1]}")

    if not args.no_gui:
        cv2.namedWindow("Trackbars")
        cv2.namedWindow("Puck Tracker")
        cv2.namedWindow("Mask")
        cv2.setMouseCallback("Puck Tracker", pick_color)

        cv2.createTrackbar("L-H", "Trackbars", 0, 179, nothing)
        cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("U-H", "Trackbars", 179, 179, nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)

        print(
            "Prompts:\n"
            " - Click the puck in 'Puck Tracker' to auto-set HSV filters.\n"
            " - Press 'Esc' to exit.\n"
            " - Optional outputs: --send-stdout and/or --udp-host <host>."
        )
    else:
        print("Running in no-gui mode.")

    frame_count = 0
    prev_detected = False
    prev_center = (-1, -1)
    prev_timestamp = None
    smoothed_vx = 0.0
    smoothed_vy = 0.0
    velocity_alpha = 0.35

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting ...")
            break

        frame = cv2.resize(frame, (args.width, args.height))
        if args.flip_horizontal and args.flip_vertical:
            frame = cv2.flip(frame, -1)
        elif args.flip_horizontal:
            frame = cv2.flip(frame, 1)
        elif args.flip_vertical:
            frame = cv2.flip(frame, 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        current_hsv_frame = hsv

        if args.no_gui:
            l_h, l_s, l_v = 0, 0, 0
            u_h, u_s, u_v = 179, 255, 255
        else:
            l_h = cv2.getTrackbarPos("L-H", "Trackbars")
            l_s = cv2.getTrackbarPos("L-S", "Trackbars")
            l_v = cv2.getTrackbarPos("L-V", "Trackbars")
            u_h = cv2.getTrackbarPos("U-H", "Trackbars")
            u_s = cv2.getTrackbarPos("U-S", "Trackbars")
            u_v = cv2.getTrackbarPos("U-V", "Trackbars")

        lower_bound = np.array([l_h, l_s, l_v])
        upper_bound = np.array([u_h, u_s, u_v])

        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        detected, center, radius = detect_puck_from_mask(mask, args.min_area)

        timestamp = time.time()

        if detected and prev_detected and prev_timestamp is not None:
            dt = timestamp - prev_timestamp
            if dt > 1e-6:
                inst_vx = (center[0] - prev_center[0]) / dt
                inst_vy = (center[1] - prev_center[1]) / dt
                smoothed_vx = velocity_alpha * inst_vx + (1.0 - velocity_alpha) * smoothed_vx
                smoothed_vy = velocity_alpha * inst_vy + (1.0 - velocity_alpha) * smoothed_vy
        elif not detected:
            smoothed_vx = 0.0
            smoothed_vy = 0.0

        packet = build_tracker_packet(
            timestamp=timestamp,
            detected=detected,
            center=center,
            radius=radius,
            width=args.width,
            height=args.height,
        )

        if args.send_stdout:
            print(json.dumps(packet), flush=True)

        if udp_socket is not None and udp_target is not None:
            udp_socket.sendto(json.dumps(packet).encode("utf-8"), udp_target)

        if detected:
            cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            intercept_x = int(max(0, min(args.width - 1, args.intercept_x_px)))
            cv2.line(frame, (intercept_x, 0), (intercept_x, args.height - 1), (200, 120, 255), 1)

            arrow_dx = smoothed_vx * args.velocity_arrow_scale
            arrow_dy = smoothed_vy * args.velocity_arrow_scale
            arrow_len = float(np.hypot(arrow_dx, arrow_dy))
            if arrow_len > 3.0:
                end_x = int(max(0, min(args.width - 1, center[0] + arrow_dx)))
                end_y = int(max(0, min(args.height - 1, center[1] + arrow_dy)))
                cv2.arrowedLine(frame, center, (end_x, end_y), (255, 200, 0), 2, tipLength=0.25)

            valid_int, int_x, int_y, tti = predict_intercept_pixel(
                center=center,
                vx=smoothed_vx,
                vy=smoothed_vy,
                intercept_x=intercept_x,
                frame_height=args.height,
                max_time_s=args.max_intercept_time,
            )
            if valid_int:
                pred_pt = (int(int_x), int(int_y))
                cv2.circle(frame, pred_pt, 7, (255, 0, 255), -1)
                cv2.line(frame, center, pred_pt, (255, 0, 255), 1)
                cv2.putText(
                    frame,
                    f"INTERCEPT y:{pred_pt[1]} t:{tti:.2f}s",
                    (10, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (255, 0, 255),
                    2,
                )
            else:
                cv2.putText(
                    frame,
                    "INTERCEPT: n/a",
                    (10, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (180, 180, 180),
                    2,
                )

            speed = float(np.hypot(smoothed_vx, smoothed_vy))
            cv2.putText(
                frame,
                f"TRACKING x:{center[0]} y:{center[1]}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"v=({smoothed_vx:.1f}, {smoothed_vy:.1f}) px/s | speed={speed:.1f}",
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 200, 0),
                2,
            )
        else:
            cv2.putText(
                frame,
                "NO PUCK DETECTED",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        frame_count += 1

        if args.max_frames > 0 and frame_count >= args.max_frames:
            break

        prev_detected = detected
        prev_center = center
        prev_timestamp = timestamp

        if not args.no_gui:
            cv2.imshow("Puck Tracker", frame)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) == 27:
                break

    cap.release()
    if udp_socket is not None:
        udp_socket.close()
    if not args.no_gui:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
