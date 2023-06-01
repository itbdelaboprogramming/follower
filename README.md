# MSD700 'Follow Me' Repo: follower package for ROS

## Prerequisite
  
1. Installed [ROS](http://wiki.ros.org/ROS/Installation). In this project, we use [ROS Noetic](http://wiki.ros.org/noetic/Installation) distribution.
2. Installed [rosserial_arduino](http://wiki.ros.org/rosserial_arduino). With this package we can connect our machine with Arduino and communicate with it using ROS.  

If you haven't installed `rosserial_arduino`, here are the step to install it.  
Open the terminal and run the following command.
```bash
sudo apt-get install ros-$ROS_DISTRO-rosserial-arduino
sudo apt-get install ros-$ROS_DISTRO-rosserial
```
For our development, because we use ROS Noetic the command that we use are 
```bash
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```
  
After installing `rosserial` for our machine, we installed `rosserial` arduino library. 
1. Open the Arduino IDE, in this case we use Arduino IDE 1.8.19 but this may work in other version as well.
2. Open **Sketch** tab then select **Include Library** and click **Manage Libraries**.
3. This will open **Library Manager** Window. Search for *rosserial* and install the library.

# How to Use  

To check the system, we will move the servo based on the position of human detected by the camera.

## Setup on Arduino Board  

Just upload [this](/MSD700_Follow_Me/Arduino/control_servo_with_ros/control_servo_with_ros.ino) sketch into the Arduino Board. In this case we will use Arduino Uno R3 Board.  

Connect the servo to the Arduino Board.  

![Arduino Board](/MSD700_Follow_Me/img/servo_wiring.png?raw=true "Arduino Wiring")

## Setup on Your Machine
  
To use ROS, we need Ubuntu installed on our machine. In this case we will use itbdelabof3 in Lab, but this code should work on any machine that use Ubuntu. Next we may test it in Jetson Xavier NX.  
  
1. Download all the code from the `msd700_ws` or just pull all the code from this repo.
2. Go to the `msd700_ws` directory. For this case we go to `/home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws`
3. If you already have a ROS workspace, just copy all the contents of `msd700_ws/src` into your Workspace `src` directory.
4. Run `catkin_make` command in terminal.

```bash
catkin_make
```

5. Source the setup file. Check the output of `catkin_make` from previous step. See the *devel space*.

```bash
Base path: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws
Source space: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/src
Build space: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/build
Devel space: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/devel
Install space: /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/install
...
```

The `setup.bash` file is located in this `Devel` folder.

```bash
source /home/itbdelabof3/allproject/GPS-Tracking/MSD700_Follow_Me/msd700_ws/devel/setup.bash
```

Everytime we open a terminal, we need to source that `setup.bash` file. To make it easier, source that file into `.bashrc` file in the root directory so everytime we open terminal, it automatically do it for us.

6. Run the `roscore`.

```bash
roscore
```

7. Run the launch file.

```bash
roslaunch human_detector_pkg follower.launch
```

If the file is successfully launch, the output at the terminal will be like this.

```bash
... logging to /home/itbdelabof3/.ros/log/a9a53544-f395-11ed-b034-7bcaf4a26fc2/roslaunch-itbdelabof3-OptiPlex-7470-AIO-210559.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://itbdelabof3-OptiPlex-7470-AIO:36521/

SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.15

NODES
  /
    camera_control (human_detector_pkg/follow_me.py)
    serial_node (rosserial_python/serial_node.py)

ROS_MASTER_URI=http://localhost:11311

process[camera_control-1]: started with pid [210596]
process[serial_node-2]: started with pid [210606]
[ WARN:0@1.673] global net_impl.cpp:174 setUpNet DNN module was not built with CUDA backend; switching to CPU
```

The video stream will pop up and display the video stream of human detector.

**Please Note** that in this test we use the default camera. If we use another device as camera (for example IntelRealsense D435i), we may change the device id at [this](/MSD700_Follow_Me/msd700_ws/src/human_detector_pkg/src/follow_me.py) file. From experince, when using IntelRealsense D435i at itbdelabof3's computer, the `device id` for camera is 8 and at Jetson Xavier NX is 4.

# Code Explanation

![ros_arduino_diagram](/MSD700_Follow_Me/img/ros_arduino_diagram.png?raw=true "ROS-Arduino")

The `/camera_control` is a ROS node that will open the device camera and run a detection algorithm and detect human in the frame of the camera stream. [Here](/MSD700_Follow_Me/human_detection/README.md) to learn more about the detection algorithm. `/camera_control` will be publishing `/rover_command` topic that contain message for commanding the Arduino.  

The `/serial_node` is a ROS Node that we run so we can access the Arduino through ROS. Just imagine that this node is the node that we upload to the Arduino Board. The `/serial_node` will subscribe `/rover_command` topic that `/camera_control` had published. The message from that topic then being translated into servo movement.

## Arduino Code

Open [this](/MSD700_Follow_Me/Arduino/control_servo_with_ros/control_servo_with_ros.ino) sketch.

```cpp
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::UInt8& cmd_msg){
  if(cmd_msg.data == 1){
    servo.write(0);
  } else if(cmd_msg.data == 2){
    servo.write(180);
  } else if(cmd_msg.data == 3){
    servo.write(90);
  }
}

ros::Subscriber<std_msgs::UInt8> sub("rover_command", servo_cb);

void setup(){
  pinMode(9, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
```

First, see this part of the code:

```cpp
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt8.h>
```

In this part, we include `Servo.h` and `ros.h` to access the Servo library and ROS library. We also expected the message from the topic that we subscribed to be in type of *8-bit unsigned integer* or `UInt8` so we included it from the *standard messages* (`std_msgs`).

```cpp
ros::NodeHandle  nh;
```

This line declares a ROS node handle object, which is used to communicate with other nodes in the ROS network.

```cpp
Servo servo;
```

This line declares a Servo object, which will be used to control the servo motor.

```cpp
void servo_cb( const std_msgs::UInt8& cmd_msg){
  if(cmd_msg.data == 1){
    servo.write(0);
  } else if(cmd_msg.data == 2){
    servo.write(180);
  } else if(cmd_msg.data == 3){
    servo.write(90);
  }
}
```

This function is a callback function that will be called whenever a new message is received on the `rover_command` topic. The function reads the message data and sets the servo position accordingly. If the message data is 1, the servo will be set to the minimum position (0 degrees). If the message data is 2, the servo will be set to the maximum position (180 degrees). If the message data is 3, the servo will be set to the middle position (90 degrees).

```cpp
ros::Subscriber<std_msgs::UInt8> sub("rover_command", servo_cb);
```

This line creates a ROS subscriber object that will listen for messages on the `rover_command` topic. Whenever a new message is received, the `servo_cb()` function will be called to handle the message.

```cpp
void setup(){
  pinMode(9, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
}
```

This function is called once at the beginning of the sketch. It initializes pin 9 as an output pin, initializes the ROS node handle, subscribes to the `rover_command` topic, and attaches the servo motor to pin 9.

```cpp
void loop(){
  nh.spinOnce();
  delay(1);
}
```

This function is called repeatedly in a loop after the `setup()` function. It calls the `spinOnce()` function of the ROS node handle, which checks for incoming messages and calls the appropriate callback functions. The `delay(1)` statement adds a small delay between iterations of the loop to prevent excessive processing.

## ROS Node Code

We write the publisher node in *Python*. Open [this](/MSD700_Follow_Me/msd700_ws/src/human_detector_pkg/src/follow_me.py) python file. The algorithm for object detection is as same as [this](/MSD700_Follow_Me/human_detection/). [Here](/MSD700_Follow_Me/human_detection/README.md) to learn more about the detection algorithm.

```python
#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import UInt8
from darknet_yolo import *
from device_camera import *

#Initialize Darknet and Camera
net = DarknetDNN()
camera = DeviceCamera(0)
start_time = time.time()
frequency = 10 #in Hz

rospy.init_node('camera_control')
pub = rospy.Publisher('rover_command', UInt8, queue_size=10)

while not rospy.is_shutdown():
    #Get frame from camera
    frame = camera.get_frame()

    #Detect human from the frame
    net.detect_object(frame)
    
    #Draw bounding box of the human detected
    net.draw_object(frame)

    #Publish the command
    if time.time() - start_time >= 1/frequency:
        direct = net.get_command()
        if direct == 'Right':
            command = 1
        elif direct == 'Left':
            command = 2
        elif direct == 'Center':
            command = 3
        else:
            command = 0
        
        rospy.loginfo(command)
        pub.publish(command)

        start_time = time.time()

    #Draw grid
    camera.create_grid()

    #Display the image
    camera.show()

    #exit condition
    key = cv2.waitKey(1)
    if key == 27:
        print(f"Key {key} is pressed.")
        break

camera.release()
```

First see this part of the code.

```python
import rospy
import time
from std_msgs.msg import UInt8
from darknet_yolo import *
from device_camera import *
```

In this part, we import `rospy` and `time` to access ROS library for python and `time` to use as timing for publishing the topic. The message that we want to publish later will be in type of *8-bit unsigned integer*, so we import `UInt8` from `std_msgs.msg`. We also import `DarknetDNN` and `DeviceCamera` to access the neural network and the camera.

```python
#Initialize Darknet and Camera
net = DarknetDNN()
camera = DeviceCamera(0)
start_time = time.time()
frequency = 10 #in Hz
```

This part we create DarknetDNN and DeviceCamera object. **Don't forget to change the device id in DeviceCamera according to the cmaera that will be used.** We also initiate start time to get timestamp for when the program will be start. We also set the frequency that the topic will be published. In this case we will use 10 Hz.

```python
rospy.init_node('camera_control')
pub = rospy.Publisher('rover_command', UInt8, queue_size=10)
```

This part of code we initializes the node with the given name 'camera_control'. This creates a new node for the ROS system. A node in ROS is a process that performs computation. A node can publish or subscribe to a certain topic and can also provide or use a certain service. A node can also interact with other nodes in the ROS system. Then we creates a new publisher that sends messages of the type `UInt8` on the topic `rover_command`. A publisher is a ROS node that sends messages on a topic. The first argument to the Publisher constructor is the topic name, the second argument is the message type, and the third argument is the queue size. The queue size determines how many messages can be stored in the publisher's buffer before old messages are discarded. In this case, the queue size is set to 10.

```python
while not rospy.is_shutdown():
    ...
    #Publish the command
    if time.time() - start_time >= 1/frequency:
        direct = net.get_command()
        if direct == 'Right':
            command = 1
        elif direct == 'Left':
            command = 2
        elif direct == 'Center':
            command = 3
        else:
            command = 0
        
        rospy.loginfo(command)
        pub.publish(command)

        start_time = time.time()
    ...
```

When publishing the topic, we first check the current time subtract it with the timestamp. We then compare it with 1/frequency that we will publish the topic. This condition ensures that the command is published at a regular frequency, determined by the frequency variable.

The direct variable is assigned a value returned from a function `net.get_command()`. This function returns a direction command, such as `Right`, `Left`, or `Center`.

The `if-elif` statements assign a numeric value to the command variable based on the direct variable. The `command` variable will be used to publish the command to the `rover_command` topic.

`rospy.loginfo(command)` logs the `command` variable value to the console for debugging purposes.

`pub.publish(command)` publishes the `command` variable value on the `rover_command` topic using the publisher created earlier. This sends the command to the rover control system.

Finally, `start_time` is set to the current time so that the next `command` is published at the appropriate time, determined by the `frequency` variable.