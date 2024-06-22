##all the comments were made by chatgpt,they should be ok ,but there might be issues,watchout##

import cv2
import numpy as np
import serial
import time

# Attempt to connect to Arduino on either '/dev/ttyUSB0' or '/dev/ttyUSB1'
arduino_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1']
arduino = None

for port in arduino_ports:
    try:
        arduino = serial.Serial(port, 9600)
        print(f"Connected to Arduino on {port}")
        break
    except serial.SerialException:
        print(f"Failed to connect to Arduino on {port}")
        continue

if arduino is None:
    print("No Arduino connected. Exiting.")
    exit()

time.sleep(2)  # Wait for the serial connection to initialize

# Initialize video capture from webcam
cap = cv2.VideoCapture("http://192.168.42.129:50123/video")

send_data = True  # Flag to control sending data to Arduino

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Get frame dimensions
    height, width, _ = frame.shape
    midpoint = (width // 2, height // 2)

    # Create a new frame for drawing
    drawing_frame = np.zeros_like(frame)
    
    # Draw axis lines from the midpoint on the drawing frame
    cv2.line(drawing_frame, (0, midpoint[1]), (width, midpoint[1]), (0, 255, 255), 2)  # Horizontal line
    cv2.line(drawing_frame, (midpoint[0], 0), (midpoint[0], height), (0, 255, 255), 2)  # Vertical line

    # Convert frame to RGB
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Define blue color range
    lower_blue = np.array([0, 0, 80])
    upper_blue = np.array([60, 60, 150])

    # Create mask for blue pixels
    mask_blue = cv2.inRange(rgb, lower_blue, upper_blue)

    # Find contours for blue ball
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Track the blue ball
    if contours_blue:
        largest_contour_blue = max(contours_blue, key=cv2.contourArea)
        M_blue = cv2.moments(largest_contour_blue)
        if M_blue["m00"] != 0:
            cx_blue = int(M_blue["m10"] / M_blue["m00"])
            cy_blue = int(M_blue["m01"] / M_blue["m00"])
            cv2.circle(drawing_frame, (cx_blue, cy_blue), 10, (255, 0, 0), -1)
            
            # Draw line from midpoint to the blue ball on the drawing frame
            cv2.line(drawing_frame, midpoint, (cx_blue, cy_blue), (255, 0, 0), 2)
            
            # Calculate relative coordinates for the blue ball
            relative_x_blue = cx_blue - midpoint[0]
            relative_y_blue = midpoint[1] - cy_blue  # Invert y to make top positive
            cv2.putText(frame, f"Blue: ({relative_x_blue}, {relative_y_blue})", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)
            
            # Send coordinates to Arduino if send_data flag is True
            if send_data:
                data_blue = f"{relative_x_blue},{relative_y_blue}\n"
                arduino.write(data_blue.encode())
                time.sleep(0.01)  # Small delay to ensure data is transferred

    # Combine the original frame with the drawing frame
    combined_frame = cv2.addWeighted(frame, 1, drawing_frame, 1, 0)

    # Show the combined frame
    cv2.imshow("Object Tracking", combined_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("w"):
        send_data = False
        print("Sending data paused.")
    elif key == ord("s"):
        send_data = True
        print("Sending data resumed.")

# Release resources
cap.release()
cv2.destroyAllWindows()

# Close Arduino connection
try:
    arduino.close()
    print("Arduino connection closed.")
except:
    print("Failed to close Arduino connection.")

