# ros_dwm1001
**A ROS package containing a hardware driver for Decawave's DWM1001 Development Board and a ROS node wrapper that exposes the sensor driver to the ROS ecosystem.**

## Overview
The hardware driver for the DWM1001 sensor is ROS-agnostic, meaning it provides a clean functional interface to the sensor that can be used on any system independent of ROS. The driver utilizes UART over a USB connection for serial communication between the sensor and OS. Additionally, the ROS node simply wraps around the driver and connects the sensor data to ROS topics.

## Prerequisites
### Software
- Ubiquity Robotics Raspberry Pi Image: https://downloads.ubiquityrobotics.com/pi.html
        - Ubuntu 16.04 (Xenial)
        - ROS Kinetic
### Hardware
- Decawave's DWM1001 Development Kit (x5 recommended: 1 tag, 4 anchors)

## Installation
### Install from Git Repository
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/ros_dwm1001.git
cd ..
sudo apt-get install -y
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make # catkin build ros_dwm1001 (if using catkin_tools)
source devel/setup.bash
rospack profile
```

## Setup

## Nodes
- `dwm1001_node`

## Topics

## Messages

## Usage
### Example
```
roslaunch ros_dwm1001 dwm1001_uwb.launch
```

## Notes

## Contact
- Author and Maintainer: Joey Yang
- Email: joeyyang.ai@gmail.com
- GitHub: https://github.com/joeyjyyang
- LinkedIn: https://www.linkedin.com/in/joey-yang
