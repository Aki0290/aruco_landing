今回使用しているパッケージ類
---
ubuntu 22.04 \

ROS2 humble \
https://docs.ros.org/en/humble/index.html \

ardupilot sitl simulation \
https://ardupilot.org/dev/docs/ros2.html \
https://ardupilot.org/dev/docs/ros2-gazebo.html \

gazebo harmonic \
https://gazebosim.org/docs/harmonic/install/ \

mavros
https://ardupilot.org/dev/docs/ros-install.html#installing-mavros \


gazeboでのシミュレーション時に実行するコマンド
---
**1つ目のターミナル**
```bash
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

ROSじゃない場合
```bash
gz sim -v4 -r iris_runway.sdf
```
**2つ目のターミナル**
mavproxyの起動
```bash
mavproxy.py --master udp:127.0.0.1:14550  --console --map
```

上のコードを実行した後に，ジンバルの向きを調節するために以下を入力

```bash
param set SERVO9_FUNCTION 59
param set SERVO10_FUNCTION 60
rc 8 1500
rc 9 1500
rc 10 1300
```

ROSじゃない場合
```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

**３つ目のターミナル**
mavrosの起動
```bash
ros2 launch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"
```

**4つ目のターミナル**
aruco_landingの起動
```bash
cd ros2_ws
colcon build --packages-select aruco_landing
source ~/ros2_ws/install/setup.bash
ros2 run aruco_landing landing_node
```






実機のときに実行するコマンド
---
