# xycar_imu_tools

E2BOX EBIMU real-vehicle publisher and IMU-based sim-to-real vehicle calibration tools.

## Real Vehicle ASUS Computer Setup

Use this checklist on the real vehicle ASUS Ubuntu computer before running the IMU nodes. This is for native Ubuntu on the vehicle computer, so the Windows/WSL `usbipd` attach step is not needed.

1. Install the basic USB/serial tools.

```bash
sudo apt update
sudo apt install -y usbutils python3-serial
```

2. Allow the current user to access serial devices.

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

If `newgrp dialout` does not refresh the permission cleanly, log out and log in again, or reboot the ASUS computer once.

3. Plug the EBIMU USB cable into the ASUS computer and check that Linux sees the CP210x USB-UART bridge.

```bash
lsusb | grep -Ei '10c4:ea60|silicon|cp210'
dmesg | tail -n 30
ls -l /dev/ttyUSB*
python3 -m serial.tools.list_ports -v
```

Expected result:

```text
10c4:ea60 Silicon Labs CP210x UART Bridge
/dev/ttyUSB0
USB VID:PID=10C4:EA60
```

If `/dev/ttyUSB0` does not appear, load the Linux CP210x driver and check again.

```bash
sudo modprobe cp210x
lsmod | grep cp210x
ls -l /dev/ttyUSB*
```

4. Optional: create a stable device name so the launch file can always use `/dev/ebimu` even if the ttyUSB number changes.

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ebimu", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-ebimu.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/ebimu
```

If there is more than one CP210x device on the car, include the sensor serial in the udev rule. The tested unit reported serial `0001`.

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ebimu", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-ebimu.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

5. Build and run the real EBIMU publisher.

```bash
source /opt/ros/humble/setup.bash
cd ~/xycar_ws
colcon build --packages-select xycar_imu_tools --symlink-install
source install/setup.bash
ros2 launch xycar_imu_tools ebimu_real.launch.py imu_port:=/dev/ttyUSB0 use_rviz:=true
```

If the stable udev name was created, use this instead.

```bash
ros2 launch xycar_imu_tools ebimu_real.launch.py imu_port:=/dev/ebimu use_rviz:=true
```

6. Confirm the ROS topics.

```bash
ros2 topic echo /imu --once
ros2 topic hz /imu
ros2 topic echo /tf --once
```

The expected `/imu` rate is around 90-100 Hz with the EBIMU configured at `460800` baud.

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
