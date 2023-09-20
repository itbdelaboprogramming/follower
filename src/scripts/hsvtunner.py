from __future__ import print_function
import cv2 as cv
import argparse

"""
Thresholding Operations using OpenCV and HSV Color Space, with this script you can tune the HVS so it will filter the color you want to detect. Make sure save the setting tune (low HSV and high HSV) to apply it in the main program (follow_me.py)

Overview:
This Python script performs thresholding operations on a video stream using OpenCV. It allows the user to interactively adjust the thresholding parameters through trackbars in real-time, making it useful for various computer vision applications.

Initialization:
- The code initializes various parameters for thresholding, such as maximum values, initial values, and window names.
- It defines callback functions for adjusting the lower and upper ranges of Hue (H), Saturation (S), and Value (V) trackbars.

Command-Line Arguments:
- The script accepts a command-line argument '--camera' to specify the camera device number. By default, it uses camera device 0.

Video Capture:
- It captures the video stream from the specified or default camera device using OpenCV's VideoCapture module.

Trackbars and Thresholding:
- The code creates two windows: 'Video Capture' for displaying the original frame and 'Object Detection' for displaying the thresholded frame.
- Trackbars are created to allow the user to adjust the lower and upper ranges of H, S, and V values, controlling the thresholding operation.
- Callback functions update the trackbar values and ensure that the upper range is always higher than the lower range.

Thresholding Operation:
- The script continuously processes frames from the camera.
- Each frame is converted to the HSV color space using OpenCV.
- Thresholding is applied to the HSV frame using the user-defined lower and upper range values.
- The original frame and the thresholded frame are displayed in real-time.

User Interaction:
- The user can interactively adjust the thresholding parameters using the trackbars in the 'Object Detection' window.
- To exit the program, the user can press 'q' or the 'Esc' key.

Overall, this code provides a user-friendly interface for experimenting with real-time thresholding operations on a video stream, making it suitable for tasks like object detection and tracking.
"""

# Initialize the parameters for thresholding
max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value

# Define window names
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

# Callback function for adjusting the lower range of Hue trackbar
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)

# Callback function for adjusting the upper range of Hue trackbar
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)

# Callback function for adjusting the lower range of Saturation trackbar
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)

# Callback function for adjusting the upper range of Saturation trackbar
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)

# Callback function for adjusting the lower range of Value trackbar
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)

# Callback function for adjusting the upper range of Value trackbar
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)


# Parse command-line arguments
parser = argparse.ArgumentParser(
    description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument(
    '--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()

# Capture the video stream from the specified or default capturing device
cap = cv.VideoCapture(args.camera)

# Create windows to display the original frame and the thresholded frame
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)

# Create the trackbars to set the range of HSV values
cv.createTrackbar(low_H_name, window_detection_name, low_H,
                  max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name, high_H,
                  max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name, low_S,
                  max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name, high_S,
                  max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name, low_V,
                  max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name, high_V,
                  max_value, on_high_V_thresh_trackbar)

# Continuously process frames from the camera until the user exits
while True:

    _, frame = cap.read()
    if frame is None:
        break

    # Convert the frame to the HSV color space
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Apply thresholding to the frame
    frame_threshold = cv.inRange(
        frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    # Display the original frame and the thresholded frame
    cv.imshow(window_capture_name, frame)
    cv.imshow(window_detection_name, frame_threshold)

    # Wait for user input to exit the program
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
