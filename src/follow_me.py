#!/usr/bin/env python3
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), "scripts"))
from scripts.device_camera import DeviceCamera
from scripts.darknet_yolo import DarknetDNN
from scripts.tracker import ObjectTracker
import cv2
import numpy as np
import time

import rospy
from follower.msg import TargetState
from geometry_msgs.msg import Twist

"""
Real-Time Object Tracking with ROS and YOLO Detection

Overview:
This Python script combines real-time object tracking with ROS (Robot Operating System) and YOLO (You Only Look Once) detection. It enables a robot to follow objects based on YOLO object detection results.

Libraries:
- The script imports necessary libraries, including OpenCV for computer vision tasks, NumPy for numerical operations, and ROS for robot control.
- Custom modules from the "scripts" directory are imported, including "DeviceCamera" for camera handling, "DarknetDNN" for YOLO detection, and "ObjectTracker" for object tracking.

ROS Node Initialization:
- The script initializes a ROS node named 'follow_me_node' for communication and control.
- ROS publishers ("target_pub" and "vel_pub") are created to send robot commands.

Robot Control Parameters:
- Parameters such as "max_speed," "max_turn," and "target_dist" are set to configure robot movement.
- These parameters control the robot's maximum speed, turning speed, and the desired following distance from the target object.

Color Range and Threshold:
- HSV color range values (low_hsv and high_hsv) are defined for object detection.
- A color threshold is set to filter objects of interest based on color.

Main Loop:
- The script enters a continuous loop for video processing and robot control.
- It captures frames from the camera and tracks objects using the "ObjectTracker."

Robot Control:
- Robot control commands are published based on the object's position and distance.
- The robot can adjust its linear and angular velocities to follow the target.
- Control commands are published at a specified frequency.

User Interaction:
- The processed video frame is displayed with object tracking results.
- The program can be exited by pressing 'q' or the 'Esc' key.

Camera Handling and Object Tracking:
- Camera initialization, frame retrieval, and object tracking are handled using the "DeviceCamera" and "ObjectTracker" classes.
- The "DarknetDNN" class is used for YOLO object detection.
- The script communicates with ROS to control the robot's movements based on object tracking.

Overall, this code integrates object tracking with robot control through ROS, enabling a robot to follow objects detected by the YOLO model.
"""


# Initialize ROS Node
rospy.init_node('follow_me_node')

# Create ROS Publishers
target_pub = rospy.Publisher('rover_command', TargetState, queue_size=1)
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Define Parameters
max_speed = rospy.get_param("~max_speed",1.0) #default is 1 m/s
max_turn = rospy.get_param("~max_turn",0.8) #default is 0.8 rad/s
target_dist = rospy.get_param("~distance",1.0) #default is 1.0 m
use_aruco = rospy.get_param("~use_aruco", True) #default is True --> for color detection set to False
camera_fps = rospy.get_param("~camera_fps", 60) #default is 60 --> for color detection set to 30
frequency  = rospy.get_param("~follow_me_node_frequency", 40) #default is 40 hz (jetson max realsense fps is 40-ish fps) --> for color detection set to 30
stop_dist = rospy.get_param("~stop_dist", 0.5) # stop distance threshold, default is 0.5 m

# Initialize Camera and Darknet
camera = DeviceCamera(4, camera_fps)
net = DarknetDNN()
tracker = ObjectTracker(2 and 3, use_aruco)
#video = cv2.VideoCapture("C:\\Users\\luthf\\Videos\\Captures\\safety_vest_video.mp4")

# Node frequency
rate = rospy.Rate(frequency) 

# Set HSV color range and color threshold for object detection
low_hsv = np.array([0, 221, 102], dtype=np.uint8)
high_hsv = np.array([73, 255, 255], dtype=np.uint8)
net.set_hsv_range(low_hsv, high_hsv)
net.set_color_threshold(34)

while not rospy.is_shutdown():
    # Get frame from camera
    frame, depth = camera.get_frame()
    
    """
    # Get the width and height of the frame
    height, width, _ = frame.shape

    # Print the width and height
    print("Width:", width)
    print("Height:", height)
    """

    #tracker.track_object(frame, net)

    #frame 640 x 480 MIL --> std::bad_alloc
    tracker.track_object_with_time(frame, net, 10.0)

    """
    #MIL
    resized_frame = cv2.resize(frame, (160, 120))
    #frame = resized_frame
    tracker.track_object_with_time(resized_frame, net, 10.0)
    """
    
    #print(tracker.get_target_position())

    # Publish the command
    msg = TargetState()
    msg.target_distance = -1.0
    msg.target_position = 0

    position = tracker.get_target_position(depth, stop_dist)
    distance = tracker.get_target_distance(depth)

    print(tracker.get_target_center(), "->", position)

    vel = Twist()
    if distance is not None:
        vel.linear.x = max(min(0.4*round((target_dist-distance)/10)*10, max_speed), -max_speed) #round the distance error into 10^1 cm order then multiplies it by a proportional factor of 0.4, then constraint it into [-max_speed, max_speed]
    else:
        vel.linear.x = 0.0

    if position == 'Right':
        msg.target_position = 1
        vel.angular.z = max_turn
    elif position == 'Left':
        msg.target_position = 2
        vel.angular.z = -max_turn
    elif position == 'Center':
        msg.target_position = 3
        vel.angular.z = 0.0
    else:
        msg.target_position = 0
        vel.linear.x = 0.0
        vel.angular.z = 0.0
    
    if distance is not None:
        msg.target_distance = distance
    
    #rospy.loginfo(msg, tracker.get_target_center(), position)
    target_pub.publish(msg)
    vel_pub.publish(vel)
    
    # debug show image
    #frame = camera.show_fps(frame)

    # Show the result
    #cv2.imshow("Video", frame)

    # Exit condition
    # key = cv2.waitKey(1)
    # if key == 27 or key == ord('q'):
    #     print(f"Key {key} is pressed")
    #     break
    rate.sleep()

camera.stop()
