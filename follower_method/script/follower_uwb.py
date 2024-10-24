#!/usr/bin/env python3
import rospy
from msd700_msgs.msg import HardwareCommand, HardwareState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import numpy as np

class UWB_Data:
    def __init__(self):
        self.dist = 0
        self.rho = 0
        self.theta = 0

class FollowerUWB:
    def __init__(self):
        # Publisher/Subscriber
        self.target_pub = rospy.Publisher('/follower/uwb_target', PoseStamped, queue_size=10)
        self.hardware_state_sub = rospy.Subscriber('/hardware_state', HardwareState, self.hardware_state_callback, queue_size=10)
    
    def hardware_state_callback(self, msg: HardwareState):
        self.dist = msg.uwb_dist
        self.rho = msg.uwb_rho
        self.theta = msg.uwb_theta

        self.publish_target()

    def publish_target(self):
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time.now()
        new_pose.pose.position.x = self.rho * np.cos(self.theta/180*np.pi)
        new_pose.pose.position.y = self.rho * np.sin(self.theta/180*np.pi)
        new_pose.pose.position.z = 0
        new_pose.pose.orientation.x = 0
        new_pose.pose.orientation.y = 0
        new_pose.pose.orientation.z = 0
        new_pose.pose.orientation.w = 1
        self.target_pub.publish(new_pose)


if __name__ == '__main__':
    rospy.init_node('follower_uwb')
    follower_uwb = FollowerUWB()
    rospy.spin()