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
from geometry_msgs.msg import Twist

# Initialize ROS Node
rospy.init_node('follow_me_node')

# Create ROS Publishers
hardware_command_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Create ROS Subscribers
ch_ultrasonic_distances = []
right_motor_pulse_delta = 0
left_motor_pulse_delta = 0
def hardware_state_callback(msg: HardwareState):
    global ch_ultrasonic_distances, right_motor_pulse_delta, left_motor_pulse_delta
    ch_ultrasonic_distances = [msg.ch_ultrasonic_distance_1, msg.ch_ultrasonic_distance_2, msg.ch_ultrasonic_distance_3, msg.ch_ultrasonic_distance_4,
                               msg.ch_ultrasonic_distance_5, msg.ch_ultrasonic_distance_6, msg.ch_ultrasonic_distance_7, msg.ch_ultrasonic_distance_8]
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
        dark_target_direction, dark_target_distance = tracker.get_dark_target_position(ch_ultrasonic_distances, lidar_distances)
        move_position, cam_position = tracker.get_target_position(depth, obs_stop_dist, 
                                                                dark_target_direction)
        distance = tracker.get_target_distance(depth, dark_target_distance)
        distance = distance if distance is not None else -1.0

        # Create msg variable
        hw_cmd_msg = HardwareCommand()
        cmd_vel_msg = Twist()

        # move command
        if move_position == 'Right':
            hw_cmd_msg.movement_command = 1
            # hw command
            hw_cmd_msg.right_motor_speed = (0 - max_turn*wheel_distance/(2.0*wheel_radius))*9.55  #in RPM
            hw_cmd_msg.left_motor_speed = (0 + max_turn*wheel_distance/(2.0*wheel_radius))*9.55   #in RPM
            # cmd_vel
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = max_turn
        elif move_position == 'Left':
            hw_cmd_msg.movement_command = 2
            # hw command
            hw_cmd_msg.right_motor_speed = (0 + max_turn*wheel_distance/(2.0*wheel_radius))*9.55  #in RPM
            hw_cmd_msg.left_motor_speed = (0 - max_turn*wheel_distance/(2.0*wheel_radius))*9.55   #in RPM
            # cmd_vel
            cmd_vel_msg.linear = 0
            cmd_vel_msg.angular.z = -max_turn
        elif move_position == 'Center' and distance > tgt_stop_dist:
            hw_cmd_msg.movement_command = 3
            # hw command
            hw_cmd_msg.right_motor_speed = (max_speed*100.0/wheel_radius - 0)*9.55  #in RPM
            hw_cmd_msg.left_motor_speed = (max_speed*100.0/wheel_radius + 0)*9.55   #in RPM
            # cmd_vel
            cmd_vel_msg.linear.x = max_speed
            cmd_vel_msg.angular.z = 0
        else:
            hw_cmd_msg.movement_command = 0
            # hw command
            hw_cmd_msg.right_motor_speed = 0  #in RPM
            hw_cmd_msg.left_motor_speed = 0   #in RPM
            # cmd_vel
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = 0
            move_position = 'Hold'
        
        print(tracker.get_target_center(), "->", move_position, "->", cam_position)
        
        # camera command
        if cam_position == 'Up':
            hw_cmd_msg.cam_angle_command = 1
        elif cam_position == 'Down':
            hw_cmd_msg.cam_angle_command = 2
        else:
            hw_cmd_msg.cam_angle_command = 0
        
        hardware_command_pub.publish(hw_cmd_msg)
        cmd_vel_pub.publish(cmd_vel_msg)
        
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