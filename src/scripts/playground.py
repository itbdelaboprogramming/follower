import cv2
import time
from darknet_yolo import DarknetDNN
from tracker import ObjectTracker

"""
Real-Time Object Tracking with YOLO Detection and OpenCV

Overview:
This Python script demonstrates real-time object tracking using YOLO (You Only Look Once) detection and OpenCV. It combines object detection and tracking to follow objects in a video stream from a camera.

Libraries:
- We use OpenCV for video capture and image processing.
- The script imports two custom modules: "DarknetDNN" for YOLO object detection and "ObjectTracker" for object tracking.

Function:
We define a utility function "constrain" to limit a value within a specified range.

Initialization:
- We initialize the YOLO object detection model using "DarknetDNN."
- A video capture object ("cap") is created to capture video from the default camera (camera ID 0).
- We create an instance of "ObjectTracker" for object tracking.
- Initialize variables for bounding box ("bbox") and tracking state ("tracking").
- We maintain counters for frame processing and FPS (Frames Per Second) calculation.

Secondary YOLO Model:
There's a mention of creating a secondary YOLO model ("net2") with various configuration settings, but the details are not provided in this code.

Main Loop:
We enter a continuous loop for video processing.
- Read a frame from the camera using "cap.read()".
- Increment the frame count.

Object Tracking:
If object tracking is enabled ("tracking" is True):
- We attempt to detect and hunt for an object using the YOLO model ("net").
- Draw bounding boxes around detected objects.
- Extract the bounding box coordinates ("bbox").
- If an object is detected and a bounding box is available:
  - Constrain the bounding box coordinates to fit within the frame.
  - Set the target for tracking using "ObjectTracker."

Object Tracking Update:
If tracking is in progress:
- We update the tracker with the current frame.
- If the update is successful, we draw a rectangle around the tracked object.

FPS Calculation:
We calculate and display the Frames Per Second (FPS) of the video stream.

User Interaction:
The processed frame with object tracking is displayed in a window named "Object."
The program can be exited by pressing 'q' or the 'Esc' key.

Cleanup:
- After exiting the loop, we release the video capture object ("cap").
- Close all OpenCV windows.

Note:
The script mentions the creation of a secondary YOLO model ("net2") with additional configurations, but these details are not provided. You may need to refer to the specific implementation of "DarknetDNN" for further insights.

Overall, this code showcases how to implement real-time object tracking by combining YOLO detection with OpenCV's tracking capabilities.
"""

# Define a function to constrain a value within a given range
def constrain(val, low, high):
    if val < low:
        return low
    elif val > high:
        return high
    else:
        return val

# Initialize the YOLO object detection model
net = DarknetDNN()


# Initialize the video capture from the camera (camera ID 0)
cap = cv2.VideoCapture(0)

# Initialize the ObjectTracker for object tracking
tr = ObjectTracker()

# Initialize variables for bounding box and tracking state
bbox = None
tracking = False

# Initialize frame count and start time for FPS calculation
frame_count = 0
start_time = time.time()

# Create a secondary YOLO object detection model (not described in code)
net2 = DN()
net2.set_hsv_range()
net2.set_color_threshold()
net2.set_confidence_threshold()
net2.set_nms_threshold()
net2.detect_object()

# Main loop for video processing
while True:

    # Read a frame from the camera
    retval, frame = cap.read()
    frame_count += 1
    
    # If object tracking is enabled
    if tracking:
        # Hunt for the object using the YOLO model
        tr.create(7)
        net.hunt(frame)
        net.draw_hunted_target(frame)
        bbox = net.get_target_bbox()
        
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            x1 = constrain(x1, 0, 640)
            y1 = constrain(y1, 0, 480)
            x2 = constrain(x2, 0, 640)
            y2 = constrain(y2, 0, 480)
            kotak = [x1, y1, x2-x1, y2-y1]
            ret = tr.set_target(frame, kotak)
            print(ret)
            tracking = True
    
    if tracking:
        # Update the tracker with the current frame
        success, box = tr.update(frame)

        if success:
            x1, y1, w, h = [int(v) for v in box]
            # Draw a rectangle around the tracked object
            cv2.rectangle(frame, (x1, y1), (x1 + w , y1 + h), (255,0,0), 2)
        else:
            # Stop tracking if the object is lost
            tr.create(0)
            tracking = False

    # Calculate and display the Frames Per Second (FPS)
    current_time = time.time()
    elapsed_time = current_time - start_time
    if elapsed_time >= 1:
        fps = round(frame_count/elapsed_time, 2)
        start_time = current_time
        frame_count = 0
        print(fps)

    # Display the processed frame with object tracking 
    cv2.imshow("Object", frame)

    # Exit condition: Press 'q' or 'Esc' to quit
    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        print(f"Key {key} is pressed")
        break

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()



