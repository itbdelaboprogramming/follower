from device_camera import DeviceCamera
from darknet_yolo import DarknetDNN
import cv2
import time

# Initialize Camera and Darknet
camera = DeviceCamera(4)
net = DarknetDNN()
#video = cv2.VideoCapture("C:\\Users\\luthf\\Videos\\Captures\\safety_vest_video.mp4")

# Time stamp
start_time = time.time()
frequency = 10 # in Hz

while True:
    # Get frame from camera
    frame, _ = camera.get_frame()

    frame = camera.show_fps(frame)

    #ret, frame = video.read()
    #if not ret:
    #    break

    # Detect the human from the frame
    #net.detect_object(frame)
    
    # Draw the bounding box of the object detected
    #net.draw_detected_object(frame)

    # Show the result
    cv2.imshow("Video", frame)

    # Exit condition
    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        print(f"Key {key} is pressed")
        break
