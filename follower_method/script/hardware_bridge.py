#!/usr/bin/env python3
import os, sys

import rospy
from ros_msd700_msgs.msg import HardwareCommand, HardwareState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FollowerHardware:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('follower_hw_bridge_node')

        # Params
        hardware_command = rospy.get_param("/follower_node/hardware_command")

        # Publisher/Subscriber
        hardware_command_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)

    
    def publish_hardware_command(self):
        hw_cmd_msg = HardwareCommand()
        # move command
        if move_position == 'Right':
            # hw command
            hw_cmd_msg.movement_command = 1
            hw_cmd_msg.right_motor_speed = (0 - max_turn*wheel_distance/(2.0*wheel_radius))*9.55  #in RPM
            hw_cmd_msg.left_motor_speed = (0 + max_turn*wheel_distance/(2.0*wheel_radius))*9.55   #in RPM
        elif move_position == 'Left':
            # hw command
            hw_cmd_msg.movement_command = 2
            hw_cmd_msg.right_motor_speed = (0 + max_turn*wheel_distance/(2.0*wheel_radius))*9.55  #in RPM
            hw_cmd_msg.left_motor_speed = (0 - max_turn*wheel_distance/(2.0*wheel_radius))*9.55   #in RPM
        elif move_position == 'Center':
            # hw command
            hw_cmd_msg.movement_command = 3
            hw_cmd_msg.right_motor_speed = (max_speed*100.0/wheel_radius - 0)*9.55  #in RPM
            hw_cmd_msg.left_motor_speed = (max_speed*100.0/wheel_radius + 0)*9.55   #in RPM
        else:
            # hw command
            hw_cmd_msg.movement_command = 0
            hw_cmd_msg.right_motor_speed = 0  #in RPM
            hw_cmd_msg.left_motor_speed = 0   #in RPM
            
            # camera command
            if cam_position == 'Up':
                hw_cmd_msg.cam_angle_command = 1
            elif cam_position == 'Down':
                hw_cmd_msg.cam_angle_command = 2
            else:
                hw_cmd_msg.cam_angle_command = 0
            
            hardware_command_pub.publish(hw_cmd_msg)

if __name__ == '__main__':
