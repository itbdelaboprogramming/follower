#!/usr/bin/env python3
import os, sys
from typing import Union
import math

import rospy
from geometry_msgs.msg import Twist, PoseStamped

class FollowerControl:
    def __init__(self):
        # Params
        self.target_topic = rospy.get_param("/follower_control/target_topic", "/follower/follower_target")
        self.cmd_vel_topic = rospy.get_param("/follower_control/cmd_vel_topic", "/cmd_vel")
        self.twist_mul = rospy.get_param("/follower_control/twist_mul", 1.0)

        # Publisher
        self.cmd_vel_pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscriber
        self.target_sub_ = rospy.Subscriber("/follower/follower_target", PoseStamped, self.target_callback, queue_size=10)
    
    def target_callback(self, msg: PoseStamped) -> None:
        # Define target pos and ort
        target_pose = msg.pose
        target_pos = target_pose.position

        rho = (target_pos.x**2 + target_pos.y**2)**0.5
        theta = math.atan2(target_pos.y, target_pos.x)
        rospy.loginfo(f"rho: {rho:.2f}, theta: {theta:.2f}")

        # Define cmd_vel
        new_twist = Twist()
        new_twist.linear.x = rho * self.twist_mul
        new_twist.angular.z = theta * self.twist_mul
        self.cmd_vel_pub_.publish(new_twist)

if __name__ == '__main__':
    rospy.init_node('follower_control')
    follower_uwb = FollowerControl()
    rospy.spin()