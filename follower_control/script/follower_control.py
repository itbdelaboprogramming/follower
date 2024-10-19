#!/usr/bin/env python3
import os, sys
from typing import Union

import rospy
from ros_msd700_msgs.msg import HardwareCommand, HardwareState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped


class FollowerControl:
    def __init__(self):
        # Publisher/Subscriber
        self.hardware_command_pub = rospy.Publisher("/hardware_command", HardwareCommand, queue_size=10)
        self.uwb_target_sub = rospy.Subscriber("/follower/uwb_target", PoseStamped, self.uwb_target_callback, queue_size=10)
    
    def uwb_target_callback(self, msg: PoseStamped):
        left_rpm, right_rpm = self.target_to_motor_speed(msg)
        new_hw_cmd = HardwareCommand()
        new_hw_cmd.movement_command = 0
        new_hw_cmd.cam_angle_command = 0
        new_hw_cmd.right_motor_speed = right_rpm
        new_hw_cmd.left_motor_speed = left_rpm
        self.hardware_command_pub.publish(new_hw_cmd)

    def target_to_motor_speed(self, target : PoseStamped) -> Union[int, int]:
        # Define target pos and ort
        target_pose = target.pose
        target_pos = target_pose.position

        # These param should be in a yaml file and need tuning
        max_speed = 1
        max_turn = 0.8
        wheel_radius = 2.75
        wheel_distance = 23.0
        err_y = 0.1

        # x is forward, y is left/right
        fwd_speed = (max_speed*100.0/wheel_radius)*9.55
        rot_speed = (max_turn*wheel_distance/(2.0*wheel_radius))*9.55

        if(abs(target_pos.y) < err_y):
            # forward/backward movement
            if(target_pos.x > 2):
                left_motor_rpm = fwd_speed
                right_motor_rpm = fwd_speed
        else:
            # rotation movement
            direction = 1 if target_pos.y > 0 else -1
            left_motor_rpm = direction*rot_speed
            right_motor_rpm = -direction*rot_speed
        
        return left_motor_rpm, right_motor_rpm
    
if __name__ == '__main__':
    rospy.init_node('follower_control')
    follower_uwb = FollowerControl()
    rospy.spin()