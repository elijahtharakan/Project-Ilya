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
    parser.add_argument("--width", type=int, default=640, help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Frame height (default: 480)")
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
    return parser.parse_args()


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


def main():
    global current_hsv_frame
    args = parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    udp_socket = None
    udp_target = None
    if args.udp_host:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_target = (args.udp_host, args.udp_port)
        print(f"UDP output enabled: {udp_target[0]}:{udp_target[1]}")

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

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting ...")
            break

        frame = cv2.resize(frame, (args.width, args.height))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        current_hsv_frame = hsv

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

        packet = build_tracker_packet(
            timestamp=time.time(),
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
            cv2.putText(
                frame,
                f"TRACKING x:{center[0]} y:{center[1]}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
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

        cv2.imshow("Puck Tracker", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) == 27:
            break

    cap.release()
    if udp_socket is not None:
        udp_socket.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
