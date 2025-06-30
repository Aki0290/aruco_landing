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
cd ros2_ws
colcon build --packages-select aruco_landing
source ~/ros2_ws/install/setup.bash
```
*Note: The ArduPilot SITL/Gazebo environment should be built separately in its own workspace (`~/ardu_ws`) according to its documentation.*

---

#### ardupilot ROS2 with SITL in GAZEBO package
```
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup
source install/setup.bash
```
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

You need to setup these parameters below.

```
param set SERVO9_FUNCTION 59
param set SERVO10_FUNCTION 60
rc 8 1500
rc 9 1500
rc 10 1300
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

## 4. Autostart on Boot (For Competition Use)

For competition or field deployment, the system is designed to launch all necessary software automatically when the Raspberry Pi is powered on. This creates a reliable, "headless" setup that is ready for flight without manual intervention. This is achieved using a standard Linux `systemd` service.

### 4.1. The Launch Script

First, a simple shell script is needed to source the correct ROS 2 environments and execute our main launch file.

1.  On your Raspberry Pi, create a file named `launch_drone.sh` in your home directory (`~/`).

    ```bash
    nano ~/launch_drone.sh
    ```

2.  Paste the following content into the file. This script ensures that the ROS environment is correctly sourced before running the launch command for the **real hardware configuration**.

    **File: `~/launch_drone.sh`**
    ```sh
    #!/bin/bash
    
    # Source the main ROS 2 environment
    source /opt/ros/humble/setup.bash
    
    # Source your project's workspace
    source ~/ros2_ws/install/setup.bash
    
    # Execute the main launch file. 
    # No arguments are needed, so it runs in real-flight mode by default.
    ros2 launch aruco_landing aruco_landing.launch.py
    ```

3.  Save the file and make it executable.
    ```bash
    chmod +x ~/launch_drone.sh
    ```

### 4.2. The `systemd` Service File

Next, we create a `systemd` service file that tells the operating system to run our script on boot.

1.  Create the service file using a text editor with `sudo` privileges.
    ```bash
    sudo nano /etc/systemd/system/drone.service
    ```

2.  Paste the following content. **Crucially, you must replace `<YOUR_USERNAME>` with your actual username on the Raspberry Pi (e.g., `pi`).**

    **File: `/etc/systemd/system/drone.service`**
    ```ini
    [Unit]
    Description=Drone Autostart Service for ERC2025
    # Start this service after the network is ready
    After=network.target
    
    [Service]
    # The user that the script will run as
    User=<YOUR_USERNAME>
    
    # The command to execute
    ExecStart=/home/<YOUR_USERNAME>/launch_drone.sh
    
    # Automatically restart the service if it fails
    Restart=on-failure
    
    [Install]
    # Enable this service for the default multi-user system state
    WantedBy=multi-user.target
    ```

### 4.3. Enabling and Managing the Service

Finally, enable the service to make it persistent across reboots.

1.  Reload the `systemd` manager to recognize the new service file.
    ```bash
    sudo systemctl daemon-reload
    ```

2.  Enable the service to start automatically on every boot.
    ```bash
    sudo systemctl enable drone.service
    ```

Your setup is now complete! The next time you power on the Raspberry Pi, your entire drone software stack will launch automatically.

> [!TIP]
> **How to check and debug your service:**
> These commands are essential for troubleshooting your autostart setup.
>
> ```bash
> # Check the current status of your service
> sudo systemctl status drone.service
>
> # Manually start your service without rebooting
> sudo systemctl start drone.service
>
> # Manually stop your service
> sudo systemctl stop drone.service
>
> # View the live log output of your service (very useful for debugging!)
> sudo journalctl -u drone.service -f
> ```
