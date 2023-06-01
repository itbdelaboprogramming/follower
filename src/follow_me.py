from scripts.device_camera import DeviceCamera
from scripts.darknet_yolo import DarknetDNN
import cv2
import time
import rospy
from std.msgs.msg import UInt8

# Initialize Camera and Darknet
camera = DeviceCamera(4)
net = DarknetDNN()
#video = cv2.VideoCapture("C:\\Users\\luthf\\Videos\\Captures\\safety_vest_video.mp4")

# Initialize ROS Node
rospy.init_node('follow_me_node')
pub = rospy.Publisher('rover_command', UInt8, queue_size=10)

# Time stamp
start_time = time.time()
frequency = 10 # in Hz

while True:
    # Get frame from camera
    frame, _ = camera.get_frame()

    #frame = camera.show_fps(frame)

    #ret, frame = video.read()
    #if not ret:
    #    break

    # Detect the human from the frame
    net.detect_object(frame)
    
    # Draw the bounding box of the object detected
    net.draw_detected_object(frame)

    # Show the result
    cv2.imshow("Video", frame)

    # Exit condition
    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        print(f"Key {key} is pressed")
        break
