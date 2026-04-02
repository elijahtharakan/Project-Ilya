<<<<<<< HEAD
import time
from dataclasses import dataclass
=======
import argparse
import json
import socket
import time
>>>>>>> 65bcabb617b8bfcf443e2eab745a8fb6b7cbb16c

import cv2
import numpy as np

<<<<<<< HEAD
from air_hockey_planner import AirHockeyPlanner, PlannerConfig, PuckMeasurement, TableBounds

# Global variable to store the latest HSV frame for the mouse callback
current_hsv_frame = None


@dataclass(frozen=True)
class CameraCalibration:
    """Simple pixel-to-planner coordinate mapping.

    This is a temporary bridge between the current vision prototype and the
    planning module. A future version should replace this with a measured
    table calibration or homography.
    """

    frame_width: int
    frame_height: int

    def pixel_to_planner(self, pixel_x: int, pixel_y: int) -> tuple[float, float]:
        planner_x = float(pixel_x)
        planner_y = (self.frame_height / 2.0) - float(pixel_y)
        return planner_x, planner_y

    def planner_to_pixel(self, planner_x: float, planner_y: float) -> tuple[int, int]:
        pixel_x = int(round(planner_x))
        pixel_y = int(round((self.frame_height / 2.0) - planner_y))
        return pixel_x, pixel_y


def build_camera_space_planner(frame_width: int, frame_height: int) -> tuple[AirHockeyPlanner, CameraCalibration]:
    """Create a planner configured for the current camera pixel space.

    For now the planner works directly in image-derived coordinates so the
    vision and planning layers can be tested together before full table
    calibration is available.
    """

    calibration = CameraCalibration(frame_width=frame_width, frame_height=frame_height)
    config = PlannerConfig(
        table=TableBounds(
            min_x=0.0,
            max_x=float(frame_width),
            min_y=-(frame_height / 2.0),
            max_y=(frame_height / 2.0),
        ),
        goal_x=40.0,
        defend_x=70.0,
        home_x=95.0,
        home_y=0.0,
        paddle_reach_x_min=25.0,
        paddle_reach_x_max=frame_width * 0.45,
        paddle_radius_margin=18.0,
        max_paddle_speed=320.0,
        max_paddle_accel=1200.0,
        attack_zone_x_max=frame_width * 0.55,
        csv_log_path=None,
    )
    return AirHockeyPlanner(config=config), calibration

def nothing(x):
    """Callback function for trackbars (does nothing)"""
    pass
=======
# Global variable to store the latest HSV frame for the mouse callback.
current_hsv_frame = None

>>>>>>> 65bcabb617b8bfcf443e2eab745a8fb6b7cbb16c

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
<<<<<<< HEAD
    frame_width = 640
    frame_height = 480
    planner, calibration = build_camera_space_planner(frame_width, frame_height)
    start_time = time.monotonic()

    # Initialize webcam. 0 is usually the default camera.
    cap = cv2.VideoCapture(0)
    
    # Check if the webcam is opened correctly
=======
    args = parse_args()

    cap = cv2.VideoCapture(args.camera)
>>>>>>> 65bcabb617b8bfcf443e2eab745a8fb6b7cbb16c
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

<<<<<<< HEAD
        # Resize the frame to reduce processing time (useful for RPi later)
        # 640x480 is a standard resolution that balances quality and speed
        frame = cv2.resize(frame, (frame_width, frame_height))
        now = time.monotonic() - start_time
        
        # Convert BGR to HSV (Hue, Saturation, Value)
        # HSV is better for color detection than RGB because it separates color info (Hue) from lighting (Value)
=======
        frame = cv2.resize(frame, (args.width, args.height))
>>>>>>> 65bcabb617b8bfcf443e2eab745a8fb6b7cbb16c
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

<<<<<<< HEAD
        planner_measurement = None
        planner_reason = "No valid puck contour this frame."

        # If any contour is found
        if contours:
            # Find the largest contour by area (assuming the puck is the largest object of that color)
            c = max(contours, key=cv2.contourArea)
            
            # Filter out small contours that might be noise
            if cv2.contourArea(c) > 500:
                # Find the minimum enclosing circle and centroid
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                else:
                    center = (int(x), int(y))
=======
        timestamp = time.time()
        packet = build_tracker_packet(
            timestamp=timestamp,
            detected=detected,
            center=center,
            radius=radius,
            width=args.width,
            height=args.height,
        )
>>>>>>> 65bcabb617b8bfcf443e2eab745a8fb6b7cbb16c

        if args.send_stdout:
            print(json.dumps(packet), flush=True)

<<<<<<< HEAD
                planner_x, planner_y = calibration.pixel_to_planner(center[0], center[1])
                contour_area = cv2.contourArea(c)
                confidence = max(0.0, min(1.0, contour_area / 5000.0))
                planner_measurement = PuckMeasurement(
                    timestamp=now,
                    x=planner_x,
                    y=planner_y,
                    confidence=confidence,
                )
                planner_reason = (
                    f"Puck detection accepted. planner_x={planner_x:.1f}, "
                    f"planner_y={planner_y:.1f}, confidence={confidence:.2f}"
                )

        planner_output = planner.update(measurement=planner_measurement, now=now)
        target_pixel_x, target_pixel_y = calibration.planner_to_pixel(
            planner_output.target_x,
            planner_output.target_y,
        )

        target_pixel_x = max(0, min(frame.shape[1] - 1, target_pixel_x))
        target_pixel_y = max(0, min(frame.shape[0] - 1, target_pixel_y))

        cv2.circle(frame, (target_pixel_x, target_pixel_y), 12, (255, 0, 0), 2)
        cv2.line(
            frame,
            (int(planner.config.defend_x), 0),
            (int(planner.config.defend_x), frame.shape[0] - 1),
            (255, 0, 0),
            1,
        )
        cv2.putText(
            frame,
            f"Mode: {planner_output.mode.value}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"Target px: ({target_pixel_x}, {target_pixel_y})",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 0, 0),
            2,
        )
        cv2.putText(
            frame,
            planner_reason[:75],
            (10, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        cv2.putText(
            frame,
            planner_output.reason[:75],
            (10, 98),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 200, 255),
            1,
        )

        # Show the original frame with tracking and the mask
        cv2.imshow('Puck Tracker', frame)
        cv2.imshow('Mask', mask)
=======
        if udp_socket is not None and udp_target is not None:
            udp_socket.sendto(json.dumps(packet).encode("utf-8"), udp_target)
>>>>>>> 65bcabb617b8bfcf443e2eab745a8fb6b7cbb16c

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
<<<<<<< HEAD
            
    # When everything done, release the capture
    planner.close()
=======

>>>>>>> 65bcabb617b8bfcf443e2eab745a8fb6b7cbb16c
    cap.release()
    if udp_socket is not None:
        udp_socket.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
