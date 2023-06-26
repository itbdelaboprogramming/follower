#!/usr/bin/env python3

from device_camera import DeviceCamera
from darknet_yolo import DarknetDNN
import cv2
import time
import numpy as np

import rospy
from std_msgs.msg import UInt8

# Initialize Camera and Darknet
camera = DeviceCamera(0)
net = DarknetDNN()
#video = cv2.VideoCapture("C:\\Users\\luthf\\Videos\\Captures\\safety_vest_video.mp4")

# Time stamp
start_time = time.time()
frequency = 10 # in Hz

rospy.init_node('follow_me_node')
pub = rospy.Publisher('rover_command', UInt8, queue_size=10)

cmd = UInt8()

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    # Get frame from camera
    frame,depth = camera.get_frame()

    lower_hsv = np.array([0, 140, 185])
    upper_hsv = np.array([30, 255, 255])

    # color_area, bbox = net.detect_with_color(frame, lower_hsv, upper_hsv)
    # frame = net.draw_target(frame, color_area, bbox)
    # ret, frame = video.read()
    # if not ret:
    #    break

    # Detect the human from the frame
    bbox, confidences, positions = net.detect_human(frame)
    areas = net.check_color(frame, bbox, lower_hsv, upper_hsv)
    #areas = []
    
    # Draw the bounding box of the object detected
    net.draw_human_info(frame, bbox, confidences, positions, areas)
    net.hunt(frame, depth, bbox, confidences, positions, areas)

    #Publish the command
    try:
        if positions[0] == 'Right':
            cmd = 1
        elif positions[0] == 'Left':
            cmd = 2
        elif positions[0] == 'Center':
            cmd = 3
        else:
            cmd = 0
    except BaseException as e:
        cmd = 0
        print(e)
        pass
        
        
    print(positions, cmd)
    pub.publish(cmd)

    start_time = time.time()

    #Draw grid
    #camera.create_grid()
    
    frame = camera.show_fps(frame)

    # Show the result
    cv2.imshow("Video", frame)

    # Exit condition
    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        print(f"Key {key} is pressed")
        break

    rate.sleep()
