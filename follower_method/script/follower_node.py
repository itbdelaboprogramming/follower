#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import numpy as np

class FollowerNode:
    def __init__(self) -> None:
        # Subscribers
        self.uwb_target_sub = rospy.Subscriber('uwb_target', PoseStamped, self.uwb_target_callback)
        self.vision_target_sub = rospy.Subscriber('vision_target', PoseStamped, self.vision_target_callback)

        # Publishers
        self.follower_target_pub = rospy.Publisher('follower_target', PoseStamped, queue_size=10)

        # Parameters
        self.debug          = rospy.get_param('follower_node/debug', False)
        self.frame_id       = rospy.get_param('follower_node/frame_id', 'map')
        self.compute_time   = rospy.get_param('follower_node/compute_time', 0.1)
        self.mode           = rospy.get_param('follower_node/mode', 'uwb')

        # Main Timer
        self.compute_timer = rospy.Timer(rospy.Duration(self.compute_time), self.main_callback)
        self.target_pose = None
    
    def main_callback(self) -> None:
        if(self.uwb_target is None and self.vision_target is None):
            return
        if(self.mode == "uwb"):
            self.target_pose = self.uwb_target
        elif(self.mode == "vision"):
            self.target_pose = self.vision_target

    def uwb_target_callback(self, msg: PoseStamped) -> None:
        self.uwb_target = msg

    def vision_target_callback(self, msg: PoseStamped) -> None:
        self.vision_target = msg
    
    def publish_pose(self) -> None:
        new_pose = PoseStamped()
        new_pose.header.frame_id = "map"
        new_pose.header.stamp = rospy.Time.now()
        new_pose.pose.position.x = self.rho * np.cos(self.theta)
        new_pose.pose.position.y = self.rho * np.sin(self.theta)
        new_pose.pose.position.z = 0
        new_pose.pose.orientation.x = 0
        new_pose.pose.orientation.y = 0
        new_pose.pose.orientation.z = 0
        new_pose.pose.orientation.w = 1
        self.target_pub.publish(new_pose)

if __name__ == '__main__':
    rospy.init_node('follower_node')
    follower_uwb = FollowerNode()
    rospy.spin()