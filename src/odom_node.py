#!/usr/bin/env python3
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), "scripts"))
import numpy as np
import time
import math
from math import sin, cos, pi

import rospy
import tf
from ros_msd700_msgs.msg import HardwareState
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3, PoseStamped
from nav_msgs.msg import Odometry

# Initialize ROS Node
rospy.init_node('odom_node')

# Create ROS Publishers
odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
odom_broadcaster = tf.TransformBroadcaster()

# Create ROS Subscribers
right_motor_pulse_delta = 0
left_motor_pulse_delta = 0
def hardware_state_callback(msg: HardwareState):
    global right_motor_pulse_delta, left_motor_pulse_delta
    right_motor_pulse_delta = msg.right_motor_pulse_delta
    left_motor_pulse_delta = msg.left_motor_pulse_delta
hardware_state_sub = rospy.Subscriber('hardware_state', HardwareState, hardware_state_callback)

# ROS Parameters (Get rosparams loaded from follower.yaml)
wheel_radius = rospy.get_param("/follower_node/wheel_radius")                # in cm
wheel_distance = rospy.get_param("/follower_node/wheel_distance")            # in cm
gear_ratio = rospy.get_param("/follower_node/gear_ratio")
revolution_radian = rospy.get_param("/follower_node/revolution_radian")
compute_period  = rospy.get_param("/follower_node/compute_period")
wheel_radius_m = wheel_radius / 100
wheel_distance_m = wheel_distance / 100

# Initialize Odom
x = 0.0
y = 0.0
th = 0.0
current_time = rospy.Time.now()
last_time = rospy.Time.now()

# Node frequency
frequency = (1/compute_period) * 1000
rate = rospy.Rate(frequency) 

try:
    while not rospy.is_shutdown():
        # Odom
        current_time = rospy.Time.now()
        delta_angle_right = right_motor_pulse_delta/gear_ratio * revolution_radian #rad
        delta_angle_left = left_motor_pulse_delta/gear_ratio * revolution_radian #rad
        dx = wheel_radius_m/2.0 * (delta_angle_right + delta_angle_left) * cos(th) #m
        dy = wheel_radius_m/2.0 * (delta_angle_right + delta_angle_left) * sin(th) #m
        dth = (delta_angle_right - delta_angle_left) * wheel_radius_m/wheel_distance_m #rad
        x += dx
        y += dy
        th += dth
        vx = dx / (current_time - last_time).to_sec() if (current_time - last_time).to_sec() > 0 else 0 # m/s
        vy = dy / (current_time - last_time).to_sec() if (current_time - last_time).to_sec() > 0 else 0 # m/s
        vth = dth / (current_time - last_time).to_sec() if (current_time - last_time).to_sec() > 0 else 0 # rad/s
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
        odom_broadcaster.sendTransform(
            (x, y, 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )
        pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose = pose
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist = twist
        odom_pub.publish(odom_msg)

        # Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "odom"
        pose_msg.pose = pose
        pose_pub.publish(pose_msg)

        last_time = current_time
        rate.sleep()
except:
    pass