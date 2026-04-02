import time
from dataclasses import dataclass

import cv2
import numpy as np

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

def pick_color(event, x, y, flags, param):
    """Mouse callback to pick color from the frame"""
    global current_hsv_frame
    if event == cv2.EVENT_LBUTTONDOWN:
        if current_hsv_frame is not None:
            pixel = current_hsv_frame[y, x]
            h_val = pixel[0]
            s_val = pixel[1]
            v_val = pixel[2]
            
            print(f"Picked HSV: {pixel}")

            # Define a tolerance for the chosen color
            # Hue wraps around 179, but we'll keep it simple for now
            h_lower = max(0, h_val - 10)
            h_upper = min(179, h_val + 10)
            s_lower = max(0, s_val - 50)
            s_upper = min(255, s_val + 50) # You can be more lenient with saturation upwards
            v_lower = max(0, v_val - 50)
            v_upper = 255 

            # Update the trackbars to the new values
            cv2.setTrackbarPos('L-H', 'Trackbars', h_lower)
            cv2.setTrackbarPos('L-S', 'Trackbars', s_lower)
            cv2.setTrackbarPos('L-V', 'Trackbars', v_lower)
            cv2.setTrackbarPos('U-H', 'Trackbars', h_upper)
            cv2.setTrackbarPos('U-S', 'Trackbars', 255) # Max saturation usually fine
            cv2.setTrackbarPos('U-V', 'Trackbars', 255) # Max value usually fine

def main():
    global current_hsv_frame
    frame_width = 640
    frame_height = 480
    planner, calibration = build_camera_space_planner(frame_width, frame_height)
    start_time = time.monotonic()

    # Initialize webcam. 0 is usually the default camera.
    cap = cv2.VideoCapture(0)
    
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # Create windows
    cv2.namedWindow('Trackbars')
    cv2.namedWindow('Puck Tracker')

    # Set mouse callback on the video window
    cv2.setMouseCallback('Puck Tracker', pick_color)
    
    # Create trackbars for color change
    # Start with wide ranges so user sees SOMETHING initially
    cv2.createTrackbar('L-H', 'Trackbars', 0, 179, nothing)
    cv2.createTrackbar('L-S', 'Trackbars', 0, 255, nothing) # Start at 0 to see everything
    cv2.createTrackbar('L-V', 'Trackbars', 0, 255, nothing) # Start at 0 to see everything
    cv2.createTrackbar('U-H', 'Trackbars', 179, 179, nothing) # Start at max
    cv2.createTrackbar('U-S', 'Trackbars', 255, 255, nothing)
    cv2.createTrackbar('U-V', 'Trackbars', 255, 255, nothing)

    print("Prompts: \n - Click on the puck in the 'Puck Tracker' window to automatically set filters.\n - Press 'Esc' to exit.")

    while True:
        # Read frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting ...")
            break

        # Resize the frame to reduce processing time (useful for RPi later)
        # 640x480 is a standard resolution that balances quality and speed
        frame = cv2.resize(frame, (frame_width, frame_height))
        now = time.monotonic() - start_time
        
        # Convert BGR to HSV (Hue, Saturation, Value)
        # HSV is better for color detection than RGB because it separates color info (Hue) from lighting (Value)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Update the global hsv frame for the mouse callback
        current_hsv_frame = hsv

        # Get current positions of all trackbars
        l_h = cv2.getTrackbarPos('L-H', 'Trackbars')
        l_s = cv2.getTrackbarPos('L-S', 'Trackbars')
        l_v = cv2.getTrackbarPos('L-V', 'Trackbars')
        u_h = cv2.getTrackbarPos('U-H', 'Trackbars')
        u_s = cv2.getTrackbarPos('U-S', 'Trackbars')
        u_v = cv2.getTrackbarPos('U-V', 'Trackbars')

        # Define range of color in HSV
        lower_bound = np.array([l_h, l_s, l_v])
        upper_bound = np.array([u_h, u_s, u_v])

        # Threshold the HSV image to get only the color within the range
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Morphological operations to remove noise
        # Erode removes small white noise
        mask = cv2.erode(mask, None, iterations=2)
        # Dilate expands the white regions (helps fill in holes)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

                # Draw the circle and centroid on the frame
                # Yellow circle around the puck
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Red dot at the center
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
                # Display coordinates on the screen
                cv2.putText(frame, f"x: {center[0]}, y: {center[1]}", (10, frame.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

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

        # Wait for 'Esc' key to stop
        key = cv2.waitKey(1)
        if key == 27:
            break
            
    # When everything done, release the capture
    planner.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
