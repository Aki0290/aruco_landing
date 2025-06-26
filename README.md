# ERC2025 UAV Autonomous Navigation
ERC2025 UAV Navigation Package

## Environment

| - | version |
|------|------------|
| Drone | PFM Zephyr |
| OS  | Ubuntu22.04 |
| ROS | ROS2 Humble |

If you did not install these environment, please access these links below and install. 

ardupilot sitl simulation \
https://ardupilot.org/dev/docs/ros2.html \
https://ardupilot.org/dev/docs/ros2-gazebo.html 

gazebo harmonic 
https://gazebosim.org/docs/harmonic/install/ 

mavros
https://ardupilot.org/dev/docs/ros-install.html#installing-mavros 

## Overview
This package enables autonomous landing on an ArUco marker, as required in the drone mission of the ERC competition.

## 1.Setup
### build the packages in your ws
#### aruco_landing package
```
cd ros2_ws
colcon build --packages-select aruco_landing
source ~/ros2_ws/install/setup.bash
```
#### ardupilot ROS2 with SITL in GAZEBO package
```
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup
source install/setup.bash
```
#### ArUco Marker Integration
To support autonomous landing on an ArUco marker, We modified the existing SDF files used in the ArduPilot with Gazebo SITL simulation. Specifically, we added an ArUco marker model into the simulation world by editing the .sdf file of the world.

## 2. Simulation in Gazebo
---
> [!IMPORTANT]
> Plz source bash files before launch .py

### launch ardupilot simulation environment

### 1. launch ardupilot package
```
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```
### 2. launch mavproxy
```
mavproxy.py --master udp:127.0.0.1:14550  --console --map
```
You need to setup these parameters below.
```
param set SERVO9_FUNCTION 59
param set SERVO10_FUNCTION 60
rc 8 1500
rc 9 1500
rc 10 1300
```

### 3. launch mavros
```
ros2 launch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"
```

### 4. launch aruco_landing package
```
ros2 run aruco_landing landing_node
```

## 3. Testing in the real world

###

