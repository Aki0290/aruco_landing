# ERC2025 UAV Autonomous Navigation
ERC2025 UAV Navigation Package

![drone](https://github.com/user-attachments/assets/2e216aca-de09-4890-a808-00fb712153e1)

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





# ERC2025 UAV Autonomous Navigation
ERC2025 UAV Navigation Package

![drone](https://github.com/user-attachments/assets/2e216aca-de09-4890-a808-00fb712153e1)

## Environment

| Item | Version / Type |
| :--- | :--- |
| Drone | PFM Zephyr |
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Companion Computer | Raspberry Pi 4 (4GB+ recommended) |

If you have not installed the required software, please refer to the official documentation:

* **ArduPilot SITL with Gazebo:** [ardupilot.org/dev/docs/ros2-gazebo.html](https://ardupilot.org/dev/docs/ros2-gazebo.html)
* **Gazebo Harmonic:** [gazebosim.org/docs/harmonic/install](https://gazebosim.org/docs/harmonic/install)
* **MAVROS for ROS 2:** [ardupilot.org/dev/docs/ros-install.html#installing-mavros](https://ardupilot.org/dev/docs/ros-install.html#installing-mavros)

## Overview
This package enables a drone to perform autonomous missions as required by the ERC competition. Key features include:
* Autonomous takeoff and landing on a specified ArUco marker.
* Detection of up to three specified objects during a search pattern.
* Calculation and logging of object positions relative to the takeoff point.
* A unified launch system that handles both simulation and real-world flight configurations.

## 1. Setup
### Build the Package
Navigate to your ROS 2 workspace and build the `aruco_landing` package.
```bash
cd ~/ros2_ws
colcon build --packages-select aruco_landing
source install/setup.bash
```
*Note: The ArduPilot SITL/Gazebo environment should be built separately in its own workspace (`~/ardu_ws`) according to its documentation.*

---

## 2. Running in Simulation
This section explains how to test the system in a SITL (Software-In-The-Loop) environment.

> [!IMPORTANT]
> In each new terminal, ensure you have sourced both your ROS 2 workspace and the ArduPilot Gazebo workspace.
> ```bash
> # In every terminal you use:
> source ~/ros2_ws/install/setup.bash
> source ~/ardu_ws/install/setup.bash
> ```

### Launching the Simulation
The simulation requires three separate terminals.

#### Terminal 1: Launch Gazebo Environment & SITL
This command starts the Gazebo world and the ArduPilot SITL vehicle instance.
```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

#### Terminal 2: Launch MAVProxy (Optional but Recommended)
MAVProxy is useful for monitoring status and sending manual commands (e.g., `mode guided`, `arm throttle`, `takeoff 1`).
```bash
mavproxy.py --master udp:127.0.0.1:14550 --console --map
```

#### Terminal 3: Launch Onboard Software
This single command starts both MAVROS and our custom node. The `sim:=true` argument configures MAVROS for the simulation.
```bash
ros2 launch aruco_landing aruco_landing.launch.py sim:=true
```

---

## 3. Running on Real Hardware
This section describes how to run the system on the actual drone with a Raspberry Pi 4.

### 3.1. Onboard Computer (Raspberry Pi 4)
On the Raspberry Pi 4 connected to your flight controller, execute this single command. It will launch MAVROS (configured for a serial connection), the RealSense camera driver, and the main landing/detection node.
```bash
ros2 launch aruco_landing aruco_landing.launch.py
```
> [!NOTE]
> This command assumes the `fcu_url` inside the launch file is set to your RPi4's correct serial port and baud rate (e.g., `"/dev/ttyAMA0:921600"`).

### 3.2. Ground Control Station (Optional)
On your laptop (connected to the same network as the drone), you can use MAVProxy to monitor the drone's status.
```bash
# Replace <RPi4_IP_Address> with your Raspberry Pi's IP address
mavproxy.py --master udpin:<RPi4_IP_Address>:14550
```

---

## 4. Autostart on Boot (For Competition)
For competition use, it is highly recommended to set up the software to launch automatically when the Raspberry Pi is powered on. This is achieved using a `systemd` service.

### 4.1. Create a Launch Script
On your Raspberry Pi 4, create a shell script in your home directory.

**File:** `~/launch_drone.sh`
```sh
#!/bin/bash

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Source your workspace
source ~/ros2_ws/install/setup.bash

# Execute the launch file for real-world flight (no 'sim:=true')
ros2 launch aruco_landing aruco_landing.launch.py
```
Make the script executable:
```bash
chmod +x ~/launch_drone.sh
```

### 4.2. Create a systemd Service File
Create a new service file in the system directory.
```bash
sudo nano /etc/systemd/system/drone.service
```
Paste the following content into the file. **Remember to replace `pi` with your actual username.**

**File:** `/etc/systemd/system/drone.service`
```ini
[Unit]
Description=Drone Autostart Service for ERC2025
After=network.target

[Service]
User=pi  # <<< REPLACE 'pi' WITH YOUR USERNAME (e.g., akihiro)
ExecStart=/home/pi/launch_drone.sh  # <<< REPLACE 'pi' WITH YOUR USERNAME
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

### 4.3. Enable the Service
Run the following commands to enable the service. It will now start automatically every time the Raspberry Pi boots up.
```bash
# Reload the systemd manager configuration
sudo systemctl daemon-reload

# Enable the service to start on boot
sudo systemctl enable drone.service
```
You can check the status of your service anytime with `sudo systemctl status drone.service`.
