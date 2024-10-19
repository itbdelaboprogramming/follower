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
        self.marker_pub = rospy.Publisher('/follower/uwb_marker', Marker, queue_size=10)
        self.target_pub = rospy.Publisher('/follower/uwb_target', PoseStamped, queue_size=10)
        self.hardware_state_sub = rospy.Subscriber('/hardware_state', HardwareState, self.hardware_state_callback, queue_size=10)

        # get param
        self.debug = rospy.get_param('follower_uwb/debug', False)
        self.frame_id = rospy.get_param('follower_uwb/frame_id', 'map')
    
    def hardware_state_callback(self, msg: HardwareState):
        self.dist = msg.uwb_dist
        self.rho = msg.uwb_rho
        self.theta = msg.uwb_theta

        self.publish_visualization()
        self.publish_target()

    def publish_target(self):
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time.now()
        new_pose.pose.position.x = self.rho * np.cos(self.theta)
        new_pose.pose.position.y = self.rho * np.sin(self.theta)
        new_pose.pose.position.z = 0
        new_pose.pose.orientation.x = 0
        new_pose.pose.orientation.y = 0
        new_pose.pose.orientation.z = 0
        new_pose.pose.orientation.w = 1
        self.target_pub.publish(new_pose)

    def publish_visualization(self):
        new_marker = Marker()
        new_marker.header.frame_id = "map"
        new_marker.header.stamp = rospy.Time.now()
        new_marker.ns = "uwb"
        new_marker.id = 0
        new_marker.type = Marker.ARROW
        new_marker.action = Marker.ADD

        p1 = Point()
        p1.x = 0
        p1.y = 0
        p1.z = 0
        p2 = Point()
        p2.x = self.rho * np.cos(self.theta)
        p2.y = self.rho * np.sin(self.theta)
        p2.z = 0
        new_marker.points.append(p1)
        new_marker.points.append(p2)

        new_marker.pose.orientation.x = 0
        new_marker.pose.orientation.y = 0
        new_marker.pose.orientation.z = 0
        new_marker.pose.orientation.w = 1
        new_marker.scale.x = 0.1
        new_marker.scale.y = 0.2
        new_marker.scale.z = 1
        new_marker.color.a = 1.0
        new_marker.color.r = 1.0
        new_marker.color.g = 0.0
        new_marker.color.b = 0.0
        self.marker_pub.publish(new_marker)
        if self.debug:
            print("Published Marker")

if __name__ == '__main__':
    rospy.init_node('follower_uwb')
    follower_uwb = FollowerUWB()
    rospy.spin()