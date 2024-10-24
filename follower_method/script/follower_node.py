#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import numpy as np

class FollowerNode:
    def __init__(self) -> None:
        # Parameters
        self.uwb_target_topic = rospy.get_param('follower_node/uwb_target_topic', '/follower/uwb_target')
        self.follower_target_topic = rospy.get_param('follower_node/follower_target_topic', '/follower/follower_target')

        # Subscribers
        self.uwb_target_sub = rospy.Subscriber(self.uwb_target_topic, PoseStamped, self.uwb_target_callback)

        # Publishers
        self.follower_target_pub = rospy.Publisher(self.follower_target_topic, PoseStamped, queue_size=10)

        # Parameters
        self.frame_id       = rospy.get_param('follower_node/frame_id', 'map')
        self.compute_time   = rospy.get_param('follower_node/compute_time', 0.1)
        self.mode           = rospy.get_param('follower_node/mode', 'uwb')

        # Main Timer
        self.compute_timer = rospy.Timer(rospy.Duration(self.compute_time), self.main_callback)
        self.target_pose = None

    def uwb_target_callback(self, msg: PoseStamped) -> None:
        self.uwb_target = msg
    
    def main_callback(self) -> None:
        if(self.uwb_target is None):
            return

        if(self.mode == "uwb"):
            self.target_pose = self.uwb_target

        # Publish the follower_target
        self.publish_target_pose()
        # Publish the visualization marker
        self.publish_arrow("/follower_viz/follower_target", 0, 0, self.target_pose.pose.position.x, self.target_pose.pose.position.y)
    
    def publish_target_pose(self) -> None:
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time.now()
        new_pose.pose = self.target_pose.pose

        self.target_pub.publish(new_pose)

    def publish_arrow(self, ns, x1, y1, x2, y2) -> None:
        new_marker = Marker()
        new_marker.header.frame_id = self.frame_id
        new_marker.header.stamp = rospy.Time.now()
        new_marker.ns = ns
        new_marker.id = 0
        new_marker.type = Marker.ARROW
        new_marker.action = Marker.ADD

        p1 = Point()
        p1.x = x1
        p1.y = x2
        p1.z = 0
        p2 = Point()
        p2.x = x2
        p2.y = y2
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
    rospy.init_node('follower_node')
    follower_uwb = FollowerNode()
    rospy.spin()