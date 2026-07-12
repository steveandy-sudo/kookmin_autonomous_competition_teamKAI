# xycar_imu_tools

E2BOX EBIMU real-vehicle publisher and IMU-based sim-to-real vehicle calibration tools.

## Real EBIMU

The tested real sensor appears as a CP210x USB-UART device.

```bash
lsusb
ls -l /dev/ttyUSB*
python3 -m serial.tools.list_ports -v
```

Run the real sensor publisher:

```bash
source /opt/ros/humble/setup.bash
cd ~/xycar_ws
colcon build --packages-select xycar_imu_tools --symlink-install
source install/setup.bash
ros2 launch xycar_imu_tools ebimu_real.launch.py use_rviz:=true
```

Defaults:

- port: `/dev/ttyUSB0`
- baudrate: `460800`
- IMU topic: `/imu`
- frame id: `imu_link`

Check:

```bash
ros2 topic echo /imu --once
ros2 topic hz /imu
ros2 topic echo /tf --once
```

## Sim/Gazebo Calibration

The Gazebo bridge launch files in `xycar_gazebo_bridge` bridge `/imu` as `sensor_msgs/msg/Imu`.
Start Gazebo, then run the bridge and calibration node:

```bash
source /opt/ros/humble/setup.bash
cd ~/xycar_ws
colcon build --packages-select xycar_gazebo_bridge xycar_imu_tools --symlink-install
source install/setup.bash
ros2 launch xycar_gazebo_bridge xycar_gazebo_bridge.launch.py
ros2 launch xycar_imu_tools imu_vehicle_spec_calibration.launch.py use_serial_imu:=false
```

Drive the car with `/xycar_motor` commands. The calibrator listens to:

- `/imu`
- `/xycar_motor`
- `/xycar_motor_bridge/debug` (`[angle_cmd, speed_cmd, steering_rad, speed_mps, yaw_rate]`)

Output topics:

```bash
ros2 topic echo /xycar_imu_tools/vehicle_spec_estimate
ros2 topic echo /xycar_imu_tools/effective_wheel_base_m
ros2 topic echo /xycar_imu_tools/steering_gain_rad_per_cmd
```

For a real vehicle calibration run, start the serial IMU too:

```bash
ros2 launch xycar_imu_tools imu_vehicle_spec_calibration.launch.py use_serial_imu:=true use_bridge_debug:=false speed_gain_mps_per_cmd:=0.08 steering_gain_rad_per_cmd:=-0.0068
```
