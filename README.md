###今回使用しているパッケージ類
---
#
###gazeboでのシミュレーション時に実行するコマンド
---
**1つ目のターミナル**

```bash
cd ros2_ws
colcon build --packages-select aruco_landing
source ~/ros2_ws/install/setup.bash
ros2 run aruco_landing landing_node
```

**2つ目のターミナル**
```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

**３つ目のターミナル**
```bash
ros2 launch mavros apm.launch fcu_url:="udp://127.0.0.1:14550@"
```

**4つ目のターミナル**
```bash
gz sim -v4 -r iris_runway.sdf
```

###実機のときに実行するコマンド
---
