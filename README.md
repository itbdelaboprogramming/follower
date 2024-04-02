# MSD700 'Follower' Package
This document is the reference to install the Follower ROS package

# Prerequisite

Jetson Xavier NX or AGX Orin with all software setups from Microsoft Teams documentation.

# How to Use

## 1. Setup the ROS Package
1. Install Git LFS (Large File Storage) for downloading ML model files (large file):
```bash
sudo apt install git-lfs
```
2. Go to your local ROS workspace. For example:
``` bash
cd ~/catkin_ws/src
```
3. Clone the repository.
``` bash
# follower repository
git clone https://github.com/itbdelaboprogramming/follower.git

# msg and srv definition repository
git clone https://github.com/itbdelaboprogramming/ros_msd700_msgs.git
```
4. Go to
```bash
cd ~/catkin_ws
```

5. Install pyrealsense2 for Intel Realsense camera.
```bash
pip3 install pyrealsense2
```

6. Compile, type `catkin_make` into the terminal and run it.
``` bash
catkin_make
```
7. **(Optional)** GitHub max file size is limited to 100 MB. To add another large file into Git LFS, use this command on your terminal:
```bash
# command format
git lfs track "*.<file_extension>"
# for example track all *.bin files into Git LFS
git lfs track "*.bin"
```

## 2. Setup the Arduino Board
Refer to the central MSD700 firmware repository [here](https://github.com/itbdelaboprogramming/firmware-msd700/tree/main).

## 3. Run the Package
Launch the node by run this command in terminal (for robot use).
``` bash
roslaunch follower follower.launch
```
Launch the node by run this command in terminal (for camera and logic test).
``` bash
roslaunch follower debug.launch
```

# API Reference

### Nodes
follower.launch:
- follower_node
- odom_node
- serial_node

debug.launch:
- follower_node

### Publishers:
| Topic             | Message Type                    | Information                                 |
|-------------------|---------------------------------|---------------------------------------------|
| /hardware_command | ros_msd700_msgs/HardwareCommand | custom command to microcontroller hardwares |
| /cmd_vel          | geometry_msgs/Twist             | velocity command                            |

### Subscribers
| Topic           | Message Type                  | Information                                 |
|-----------------|-------------------------------|---------------------------------------------|
| /hardware_state | ros_msd700_msgs/HardwareState | custom state from microcontroller hardwares |
| /scan           | sensor_msgs/LaserScan         | lidar scan topic                            |

### ROS Parameters (rosparam):
All robot parameters are defined in `/config/follower.yaml`. Please refer to the file for more information. Rviz config is in `/config/follower.rviz`.

<br>

# Troubleshooting
- If received error `RuntimeError: Couldn't resolve requests`. Change FPS from 60 to 30 in `follower.yaml`. This is usually due to realsense USB does not support high speed data transfer.

<br>

# Important Informations
There are 6 ML-based tracker algorithms that can be used, which are:
```bash
    1. DasiamRPN
    2. CSRT
    3. KCF
    4. GOTURN
    5. MIL
    6. Nano
```


###  `hsvtunner.py` : program to calibrate the lower and upper limit values of hsv.
To tune hsv value:
```bash
cd ~/follower/src/scripts
```
then run the code
```bash
python hsvtunner.py --camera <your camera device number from /dev/video>
```
for example if the camera device number is 4 (from /dev/video4s)
```bash
python hsvtunner.py --camera 4
```
if the program run perfectly you will see 2 different camera views. One is the normal camera, and the other is the HSV filtered view result.

|            Normal Camera             |              HSV Filter              |
|:------------------------------------:|:------------------------------------:|
| ![Success](./hsv_result/Normal.png)  |![Success](./hsv_result/HSVFilter.png)|

You can adjust value for lower limit and upper limit of the hsv value until the collor that you want will be displayed by the filter camera in white while other colors will be black. Note the value of the minimum value and maximum vale of hsv, then input the value to the `low_hsv` and `high_hsv` variable at `follow_me.py`.
```bash
low_hsv = np.array(['low_H', 'low_S', 'low_V'], dtype=np.uint8)
high_hsv = np.array(['High_H', 'High_S', 'High_V'], dtype=np.uint8)
```
for example if the value of low H = 0, S = 221, V = 102 and the value of high H = 73, S = 255, V = 255 then
```bash
low_hsv = np.array([0, 221, 102], dtype=np.uint8)
high_hsv = np.array([73, 255, 255], dtype=np.uint8)
```
you also can search the limit of low and high hsv for your color in google.