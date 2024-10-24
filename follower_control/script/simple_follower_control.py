#!/usr/bin/env python3
import os, sys
from typing import Union
import math

import rospy
from ros_msd700_msgs.msg import HardwareCommand, HardwareState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped


class FollowerControl:
    def __init__(self):
        self.movement_mode = rospy.get_param("/simple_follower_control/movement_mode", "const")
        self.twist_mul = rospy.get_param("/simple_follower_control/twist_mul", 1.0)
        self.max_fwd = rospy.get_param("/simple_follower_control/max_fwd", 1.0) # m/s
        self.max_rot = rospy.get_param("/simple_follower_control/max_rot", 0.8) # rad/s
        self.max_fwd_rpm = rospy.get_param("/simple_follower_control/max_fwd_rpm", 180)
        self.max_rot_rpm = rospy.get_param("/simple_follower_control/max_rot_rpm", 70)
        self.kp_fwd = rospy.get_param("/simple_follower_control/kp_fwd", 0.1)
        self.kp_rot = rospy.get_param("/simple_follower_control/kp_rot", 0.3)
        self.err_fwd = rospy.get_param("/simple_follower_control/err_fwd", 1)
        self.err_rot = rospy.get_param("/simple_follower_control/err_rot", 0.5)

        self.wheel_radius = rospy.get_param("/simple_follower_control/wheel_radius", 2.75) / 100.0
        self.wheel_distance = rospy.get_param("/simple_follower_control/wheel_distance", 23.0) / 100.0
        self.gear_ratio = rospy.get_param("/simple_follower_control/gear_ratio", 1.0)

        rospy.loginfo(f"const:{self.const_speed}, max_fwd: {self.max_fwd}, max_rot: {self.max_rot}")
        rospy.loginfo(f", kp_fwd: {self.kp_fwd}, kp_rot: {self.kp_rot}, err_fwd: {self.err_fwd}, err_rot: {self.err_rot}")

        # Publisher/Subscriber
        self.hardware_command_pub = rospy.Publisher("/hardware_command", HardwareCommand, queue_size=10)
        self.uwb_target_sub = rospy.Subscriber("/follower/uwb_target", PoseStamped, self.uwb_target_callback, queue_size=10)
        self.vision_target_sub = rospy.Subscriber("/follower/vision_target", PoseStamped, self.uwb_target_callback, queue_size=10)
    
    def uwb_target_callback(self, msg: PoseStamped) -> None:
        left_rpm, right_rpm = self.target_to_motor_pwm(msg)
        new_hw_cmd = HardwareCommand()
        new_hw_cmd.movement_command = 0
        new_hw_cmd.cam_angle_command = 0
        new_hw_cmd.right_motor_speed = right_rpm
        new_hw_cmd.left_motor_speed = left_rpm
        self.hardware_command_pub.publish(new_hw_cmd)

    def target_to_motor_rpm(self, target : PoseStamped) -> Union[int, int]:
        # Define target pos and ort
        target_pose = target.pose
        target_pos = target_pose.position

        rho = (target_pos.x**2 + target_pos.y**2)**0.5
        theta = math.atan2(target_pos.y, target_pos.x)
        rospy.loginfo(f"rho: {rho:.2f}, theta: {theta:.2f}")

        # m/s * s/min / (m)
        self.max_fwd_rpm = self.max_fwd * 60.0 / (2.0*math.pi*self.wheel_radius * self.gear_ratio)
        self.max_fwd_rpm *= 200
        self.max_rot_rpm = (self.max_rot*self.wheel_distance/(2.0*self.wheel_radius))*9.55

        if self.const_speed:
            fwd_rpm = self.max_fwd_rpm 
            rot_rpm = self.max_rot_rpm
        else:
            fwd_rpm = min(self.max_fwd_rpm, self.kp_fwd*rho)
            rot_rpm = min(self.max_rot_rpm, self.kp_rot*theta)

        # Robot doesn't move
        left_motor_rpm = 0
        right_motor_rpm = 0
        
        # If robot facing target
        if(abs(theta) < self.err_rot):
            # forward  movement
            if(rho > self.err_fwd):
                # direction = abs(rho) / rho
                # left_motor_rpm = direction*fwd_rpm
                # right_motor_rpm = direction*fwd_rpm
                left_motor_rpm = fwd_rpm
                right_motor_rpm = fwd_rpm
                rospy.loginfo(f"Forward")
                rospy.loginfo(f"{fwd_rpm}")

        else:
            # rotation movement
            direction = abs(theta) / theta

            left_motor_rpm = direction*rot_rpm
            right_motor_rpm = -direction*rot_rpm
        
        rospy.loginfo(f"L_rpm: {left_motor_rpm:.2f}, R_rpm: {right_motor_rpm:.2f}")
        
        return left_motor_rpm, right_motor_rpm
    
if __name__ == '__main__':
    rospy.init_node('follower_control')
    follower_uwb = FollowerControl()
    rospy.spin()