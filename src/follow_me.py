#!/usr/bin/env python3
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), "scripts"))
from scripts.device_camera import DeviceCamera
from scripts.darknet_yolo import DarknetDNN
from scripts.tracker import ObjectTracker
import cv2
import numpy as np
import time

#import rospy
#from std_msgs.msg import UInt8
#from follower.msg import TargetState

# Initialize Camera and Darknet
camera = DeviceCamera(0)
net = DarknetDNN()
tracker = ObjectTracker(4)
#video = cv2.VideoCapture("C:\\Users\\luthf\\Videos\\Captures\\safety_vest_video.mp4")

# Initialize ROS Node
#rospy.init_node('follow_me_node')
#pub = rospy.Publisher('rover_command', TargetState, queue_size=10)

# Time stamp
start_time = time.time()
frequency = 10 # in Hz

low_hsv = np.array([0, 221, 102], dtype=np.uint8)
high_hsv = np.array([73, 255, 255], dtype=np.uint8)

net.set_hsv_range(low_hsv, high_hsv)
net.set_color_threshold(0)

while True: #not rospy.is_shutdown():
    # Get frame from camera
    frame, depth = camera.get_frame()

    tracker.track_object(frame, net)
    print(tracker.get_target_position())

    #ret, frame = video.read()
    #if not ret:
    #    break

    # Detect the human from the frame
    #net.detect_object(frame)
    #net.hunt(frame, depth)
    
    # Draw the bounding box of the object detected
    #net.draw_detected_object(frame)
    #net.draw_hunted_target(frame)

    # Publish the command
    """
    if time.time() - start_time >= 1/frequency:
        msg = TargetState()
        msg.target_distance = -1.0
        msg.target_position = 0

        position = net.get_target_position()
        distance = net.get_target_distance()
        
        if position == 'Right':
            msg.target_position = 1
        elif position == 'Left':
            msg.target_position = 2
        elif position == 'Center':
            msg.target_position = 3
        else:
            msg.target_position = 0
        
        if distance is not None:
            msg.target_distance = distance

        rospy.loginfo(msg)
        pub.publish(msg)

        start_time = time.time()
    """
    frame = camera.show_fps(frame)

    # Show the result
    cv2.imshow("Video", frame)

    # Exit condition
    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        print(f"Key {key} is pressed")
        break

camera.stop()