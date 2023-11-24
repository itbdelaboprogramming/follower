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
from follower.msg import HardwareState
from geometry_msgs.msg import Twist

"""
Real-Time Object Tracking with ROS and YOLO Detection

Overview:
This Python script combines real-time object tracking with ROS (Robot Operating System) and YOLO (You Only Look Once) detection. It enables a robot to follow objects based on YOLO object detection results.

Libraries:
- The script imports necessary libraries, including OpenCV for computer vision tasks, NumPy for numerical operations, and ROS for robot control.
- Custom modules from the "scripts" directory are imported, including "DeviceCamera" for camera handling, "DarknetDNN" for YOLO detection, and "ObjectTracker" for object tracking.

Parameters:
- All robot params are defined in the follower.yaml file inside /config directory

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

# ROS Parameters (Get rosparams loaded from follower.yaml)
camera_id = rospy.get_param("/camera_id")
camera_fps = rospy.get_param("/camera_fps")
use_realsense = rospy.get_param("/use_realsense")
max_speed = rospy.get_param("/max_speed") 
max_turn = rospy.get_param("/max_turn") 
tgt_stop_dist = rospy.get_param("/tgt_stop_dist")
obs_stop_dist = rospy.get_param("/obs_stop_dist")
compute_period  = rospy.get_param("/compute_period")
use_aruco = rospy.get_param("/use_aruco") 
track_algorithm = rospy.get_param("/track_algorithm")
use_debug = rospy.get_param("/use_debug", False)

# Initialize Camera and Darknet
camera = DeviceCamera(camera_id, camera_fps, use_realsense)
net = DarknetDNN()
tracker = ObjectTracker(track_algorithm, use_aruco)
#video = cv2.VideoCapture("C:\\Users\\luthf\\Videos\\Captures\\safety_vest_video.mp4")

# Node frequency
frequency = (1/compute_period) * 1000
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

    move_position, cam_position = tracker.get_target_position(depth, obs_stop_dist)
    distance = tracker.get_target_distance(depth)
    if distance is not None:
        msg.target_distance = distance
    else:
        distance = msg.target_distance

    vel = Twist()
    if distance is not None:
        vel.linear.x = max(min(0.4*round((tgt_stop_dist-distance)/10)*10, max_speed), -max_speed) #round the distance error into 10^1 cm order then multiplies it by a proportional factor of 0.4, then constraint it into [-max_speed, max_speed]
    else:
        vel.linear.x = 0.0

    # move command
    if move_position == 'Right':
        msg.target_position = 1
        vel.angular.z = max_turn
    elif move_position == 'Left':
        msg.target_position = 2
        vel.angular.z = -max_turn
    elif move_position == 'Center' and distance > tgt_stop_dist:
        msg.target_position = 3
        vel.angular.z = 0.0
    else:
        msg.target_position = 0
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        move_position = 'Hold'
    
    print(tracker.get_target_center(), "->", move_position, "->", cam_position)
    
    # camera command
    if cam_position == 'Up':
        msg.cam_angle_command = 1
    elif cam_position == 'Down':
        msg.cam_angle_command = 2
    else:
        msg.cam_angle_command = 0
    
    #rospy.loginfo(msg, tracker.get_target_center(), position)
    target_pub.publish(msg)
    vel_pub.publish(vel)
    
    if (use_debug):
        frame = camera.show_fps(frame)

        # Show the result
        cv2.imshow("Video", frame)

        # Exit condition
        key = cv2.waitKey(1)
        if key == 27 or key == ord('q'):
            print(f"Key {key} is pressed")
            break
    rate.sleep()

camera.stop()
