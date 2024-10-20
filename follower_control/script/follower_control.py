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
        self.const_speed = rospy.get_param("/follower_control/constant_speed", False)
        self.max_fwd = rospy.get_param("/follower_control/max_fwd", 0.5)
        self.max_rot = rospy.get_param("/follower_control/max_rot", 0.5)
        self.kp_fwd = rospy.get_param("/follower_control/kp_fwd", 0.1)
        self.kp_rot = rospy.get_param("/follower_control/kp_rot", 0.3)
        self.err_fwd = rospy.get_param("/follower_control/err_fwd", 0.1)
        self.err_rot = rospy.get_param("/follower_control/err_rot", 0.1)

        # Publisher/Subscriber
        self.hardware_command_pub = rospy.Publisher("/hardware_command", HardwareCommand, queue_size=10)
        self.uwb_target_sub = rospy.Subscriber("/follower/uwb_target", PoseStamped, self.uwb_target_callback, queue_size=10)
    
    def uwb_target_callback(self, msg: PoseStamped) -> None:
        left_pwm, right_pwm = self.target_to_motor_pwm(msg)
        new_hw_cmd = HardwareCommand()
        new_hw_cmd.movement_command = 0
        new_hw_cmd.cam_angle_command = 0
        new_hw_cmd.right_motor_speed = right_pwm * 100
        new_hw_cmd.left_motor_speed = left_pwm * 100
        self.hardware_command_pub.publish(new_hw_cmd)

    def target_to_motor_pwm(self, target : PoseStamped) -> Union[int, int]:
        # Define target pos and ort
        target_pose = target.pose
        target_pos = target_pose.position

        # x y is coordinate, i need plar
        rho = (target_pos.x**2 + target_pos.y**2)**0.5
        theta = math.atan2(target_pos.y, target_pos.x)

        # x is forward, y is left/right
        if not self.const_speed:
            fwd_speed = min(self.max_fwd , abs(self.kp_fwd * rho))
            rot_speed = min(self.max_rot , abs(self.kp_rot * theta))
        else:
            fwd_speed = self.max_fwd
            rot_speed = self.max_rot

        # Robot doesn't move
        left_motor_pwm = 0
        right_motor_pwm = 0
        
        # If robot facing target
        if(abs(theta) < self.err_rot):
            # forward/backward movement
            if(abs(rho) > self.err_fwd):
                direction = abs(rho) / rho
                left_motor_pwm = fwd_speed
                right_motor_pwm = fwd_speed
        else:
            # rotation movement
            direction = abs(theta) / theta
            left_motor_pwm = direction*rot_speed
            right_motor_pwm = -direction*rot_speed
        
        rospy.loginfo(f"left_motor_pwm: {left_motor_pwm:.2f}, right_motor_pwm: {right_motor_pwm:.2f}")
        
        return left_motor_pwm, right_motor_pwm
    
if __name__ == '__main__':
    rospy.init_node('follower_control')
    follower_uwb = FollowerControl()
    rospy.spin()