1つ目のターミナル
cd ros2_ws mavros 
colcon build --packages-select aruco_landing
source ~/ros2_ws/install/setup.bash
ros2 run aruco_landing landing_node


2つ目のターミナル
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

3つ目のターミナル
ros2 launch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"

4つ目のターミナル
gz sim -v4 -r iris_runway.sdf
