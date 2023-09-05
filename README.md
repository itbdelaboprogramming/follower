# MSD700 'Follower' Package

## Prerequisite

1. Installed [ROS](http://wiki.ros.org/ROS/Installation). In this project, we use [ROS Noetic](http://wiki.ros.org/noetic/Installation) distribution.
2. Installed [rosserial_arduino](http://wiki.ros.org/rosserial_arduino). With this package we can connect our machine with Arduino and communicate with it using ROS.
3. Installed [Git](https://git-scm.com/downloads).

### instal `rosserial_arduino`

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


## How to Use

### Setup the ROS Package

1. Go to your local ROS workspace. For example:
``` bash
cd ~/catkin_ws/src
```
2. Clone the repository
``` bash
git clone https://github.com/itbdelaboprogramming/follower.git
```
3. Make the ROS workspace
``` bash
catkin_make
```

### Setup the Arduino Board

1. Run the ROS Environment
``` bash
roscore
```
2. Go to your Arduino libraries directory and remove the ros_lib folder if it already exist.
``` bash
cd <sketchbook>/libraries
rm -rf ros_lib
```
3. Open a new terminal an create a new ros_lib. A new folder named `ros_lib` should appear in your Arduino Libraries directory.
``` bash
rosrun rosserial_arduino make_libraries.py .
```
4. Go back to the `follower` directory and go to the Arduino folder.
``` bash
cd ~/catkin_ws/src/follower/Arduino/test_custom_ros_msg
```
5. Open the Arduino sketch with Arduino IDE and upload it to your board.
``` bash
arduino test_custom_ros_msg
```

### Run the Package
Launch the node by run this command in terminal
``` bash
roslaunch follower follower.launch
```