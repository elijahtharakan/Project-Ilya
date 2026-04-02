import cv2
import numpy as np

# Global variable to store the latest HSV frame for the mouse callback
current_hsv_frame = None

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
        frame = cv2.resize(frame, (640, 480))
        
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

        # Show the original frame with tracking and the mask
        cv2.imshow('Puck Tracker', frame)
        cv2.imshow('Mask', mask)

        # Wait for 'Esc' key to stop
        key = cv2.waitKey(1)
        if key == 27:
            break
            
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
