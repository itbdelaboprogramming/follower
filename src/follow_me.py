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
from ros_msd700_msgs.msg import HardwareCommand, HardwareState
from sensor_msgs.msg import LaserScan

"""
Real-Time Object Tracking with ROS and YOLO Detection

Overview:
This Python script combines real-time object tracking with ROS (Robot Operating System), Aruco sign detection, and YOLO (You Only Look Once) detection. 
It enables a robot to follow objects based on the detected objects (Aruco sign or YOLO person & color).

Libraries:
- The script imports necessary libraries, including OpenCV for computer vision tasks, NumPy for numerical operations, and ROS for robot control.
- Custom modules from the "scripts" directory are imported, including "DeviceCamera" for camera handling, "DarknetDNN" for YOLO detection, and "ObjectTracker" for object tracking.

Parameters:
- All robot params are defined in the follower.yaml file inside /config directory

ROS Node Initialization:
- The script initializes a ROS node named 'follow_me_node' for communication and control.
- ROS publishers ("hardware_command_pub") are created to send robot commands.
- ROS subscriber "hardware_state_sub" is created to receive robot hardware state from Arduino.

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
- Control commands are published at a specified compute period rate each loop.

User Interaction:
- The processed video frame is displayed with object tracking results.
- For enabled debug: The program can be exited by pressing 'q' or the 'Esc' key.

Camera Handling and Object Tracking:
- Camera initialization, frame retrieval, and object tracking are handled using the "DeviceCamera" and "ObjectTracker" classes.
- The "DarknetDNN" class is used for YOLO object detection.
- The script communicates with ROS to control the robot's movements based on object tracking.

Overall, this code integrates object tracking with robot control through ROS, enabling a robot to follow objects detected by Aruco detector or YOLO model.
"""


# Initialize ROS Node
rospy.init_node('follow_me_node')

# Create ROS Publishers
hardware_command_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)

# Create ROS Subscribers
ch_ultrasonic_distance_1 = 0.0
ch_ultrasonic_distance_2 = 0.0
ch_ultrasonic_distance_3 = 0.0
ch_ultrasonic_distance_4 = 0.0
ch_ultrasonic_distance_5 = 0.0
ch_ultrasonic_distance_6 = 0.0
ch_ultrasonic_distance_7 = 0.0
ch_ultrasonic_distance_8 = 0.0
ch_ultrasonic_distances = []
right_motor_pulse_delta = 0
left_motor_pulse_delta = 0
def hardware_state_callback(msg: HardwareState):
    global ch_ultrasonic_distances, right_motor_pulse_delta, left_motor_pulse_delta, \
    ch_ultrasonic_distance_1, ch_ultrasonic_distance_2, ch_ultrasonic_distance_3, ch_ultrasonic_distance_4, \
    ch_ultrasonic_distance_5, ch_ultrasonic_distance_6, ch_ultrasonic_distance_7, ch_ultrasonic_distance_8
    ch_ultrasonic_distance_1 = msg.ch_ultrasonic_distance_1
    ch_ultrasonic_distance_2 = msg.ch_ultrasonic_distance_2
    ch_ultrasonic_distance_3 = msg.ch_ultrasonic_distance_3
    ch_ultrasonic_distance_4 = msg.ch_ultrasonic_distance_4
    ch_ultrasonic_distance_5 = msg.ch_ultrasonic_distance_5
    ch_ultrasonic_distance_6 = msg.ch_ultrasonic_distance_6
    ch_ultrasonic_distance_7 = msg.ch_ultrasonic_distance_7
    ch_ultrasonic_distance_8 = msg.ch_ultrasonic_distance_8
    ch_ultrasonic_distances = [ch_ultrasonic_distance_1, ch_ultrasonic_distance_2, ch_ultrasonic_distance_3, ch_ultrasonic_distance_4,
                               ch_ultrasonic_distance_5, ch_ultrasonic_distance_6, ch_ultrasonic_distance_7, ch_ultrasonic_distance_8]
    right_motor_pulse_delta = msg.right_motor_pulse_delta
    left_motor_pulse_delta = msg.left_motor_pulse_delta
hardware_state_sub = rospy.Subscriber('hardware_state', HardwareState, hardware_state_callback)

lidar_distances = []
def Angle2Index(laser_scan_msg, angle):
    return (int)((angle-laser_scan_msg.angle_min)/laser_scan_msg.angle_increment)
def Index2Angle(laser_scan_msg, index):
    return (laser_scan_msg.angle_min + (index*laser_scan_msg.angle_increment))
def lidar_scan_callback(msg: LaserScan):
    global lidar_distances
    lidar_distances = msg.ranges
lidar_scan_sub = rospy.Subscriber("scan", LaserScan, lidar_scan_callback)

# ROS Parameters (Get rosparams loaded from follower.yaml)
camera_id = rospy.get_param("/follower_node/camera_id")
camera_fps = rospy.get_param("/follower_node/camera_fps")
use_realsense = rospy.get_param("/follower_node/use_realsense")
max_speed = rospy.get_param("/follower_node/max_speed")                      # in m/s
max_turn = rospy.get_param("/follower_node/max_turn")                        # in rad/s
wheel_radius = rospy.get_param("/follower_node/wheel_radius")                # in cm
wheel_distance = rospy.get_param("/follower_node/wheel_distance")            # in cm
tgt_stop_dist = rospy.get_param("/follower_node/tgt_stop_dist")
obs_stop_dist = rospy.get_param("/follower_node/obs_stop_dist")
compute_period  = rospy.get_param("/follower_node/compute_period")
use_aruco = rospy.get_param("/follower_node/use_aruco")
aruco_id = rospy.get_param("/follower_node/aruco_id")
track_algorithm = rospy.get_param("/follower_node/track_algorithm")
enable_transducer = rospy.get_param("/follower_node/enable_transducer")
use_debug = rospy.get_param("/use_debug", False)

# Initialize Camera and Darknet
camera = DeviceCamera(camera_id, camera_fps, use_realsense)
net = DarknetDNN()
tracker = ObjectTracker(track_algorithm, use_aruco, aruco_id, enable_transducer)

# Node frequency
frequency = (1/compute_period) * 1000
rate = rospy.Rate(frequency) 

# Set HSV color range and color threshold for object detection
low_hsv = np.array([0, 221, 102], dtype=np.uint8)
high_hsv = np.array([73, 255, 255], dtype=np.uint8)
net.set_hsv_range(low_hsv, high_hsv)
net.set_color_threshold(34)

# Main Loop
try:
    while not rospy.is_shutdown():
        # Get frame from camera
        frame, depth = camera.get_frame()

        # track object
        tracker.track_object_with_time(frame, net, 10.0)
        dark_target_direction, dark_target_distance = tracker.get_dark_area_target(ch_ultrasonic_distances, lidar_distances)
        move_position, cam_position = tracker.get_target_position(depth, obs_stop_dist, 
                                                                dark_target_direction)
        distance = tracker.get_target_distance(depth, dark_target_distance)
        distance = distance if distance is not None else -1.0

        # Create msg variable
        msg = HardwareCommand()
        msg.movement_command = 0

        # move command
        if move_position == 'Right':
            msg.movement_command = 1
            # inverse kinematics
            msg.right_motor_speed = (0 - max_turn*wheel_distance/(2.0*wheel_radius))*9.55  #in RPM
            msg.left_motor_speed = (0 + max_turn*wheel_distance/(2.0*wheel_radius))*9.55   #in RPM
        elif move_position == 'Left':
            msg.movement_command = 2
            msg.right_motor_speed = (0 + max_turn*wheel_distance/(2.0*wheel_radius))*9.55  #in RPM
            msg.left_motor_speed = (0 - max_turn*wheel_distance/(2.0*wheel_radius))*9.55   #in RPM
        elif move_position == 'Center' and distance > tgt_stop_dist:
            msg.movement_command = 3
            msg.right_motor_speed = (max_speed*100.0/wheel_radius - 0)*9.55  #in RPM
            msg.left_motor_speed = (max_speed*100.0/wheel_radius + 0)*9.55   #in RPM
        else:
            msg.movement_command = 0
            msg.right_motor_speed = 0  #in RPM
            msg.left_motor_speed = 0   #in RPM
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
        hardware_command_pub.publish(msg)
        
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

except:
    camera.stop()