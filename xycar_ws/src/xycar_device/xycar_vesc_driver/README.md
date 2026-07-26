# Native ROS 2 Xycar VESC driver

This package replaces the complete legacy motor path:

```text
ROS 2 /xycar_motor
  -> ros1_bridge
  -> ROS 1 xycar_motor.py
  -> ROS 1 ackermann_to_vesc
  -> ROS 1 vesc_driver
  -> /dev/ttyMOTOR
```

with:

```text
ROS 2 /xycar_motor
  -> xycar_vesc_driver
  -> /dev/ttyMOTOR
```

The external command remains
`std_msgs/msg/Float32MultiArray [angle_command, speed_command]`.
The legacy conversion constants are preserved. The node directly publishes
`/vehicle/vesc_state`, `/vehicle/system_telemetry`, `/odom`,
`/xycar_motor_bridge/debug`, `/tf`, and `/diagnostics`.

## Safety defaults

- `drive_enabled` is false.
- firmware must be 2.18;
- commands stop after a 0.5 second publisher timeout;
- acceleration is slew-limited;
- fresh VESC telemetry is required;
- acceleration is inhibited below 7.5 V;
- propulsion is reduced linearly from 7.5 V to zero at 6.0 V;
- output is latched off at 6.0 V or on any VESC fault;
- a 6.0 V latch or VESC `UNDER_VOLTAGE` fault clears automatically only after
  the fault clears and voltage stays at or above 8.0 V for three seconds;
- other VESC hardware faults require inspection and the clear service after
  stable recovery voltage is available.

Do not lower the voltage thresholds to work around a weak battery or a
high-resistance connector.

## Build

```bash
cd ~/kookmin_ty/slam_gazebo_controller/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select xycar_msgs xycar_vesc_driver
source install/setup.bash
```

## First start: output disabled

Stop the ROS 1 motor container and dynamic bridge first. Only one process may
own `/dev/ttyMOTOR`.

```bash
ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py
ros2 topic echo /vehicle/vesc_state
ros2 topic echo /diagnostics
```

Confirm the firmware version, approximately 50 Hz VESC telemetry, fault code
zero, and correct voltage before enabling output.

## Wheels-off-ground validation

Keep a physical emergency stop available.

```bash
ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  drive_enabled:=true

ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 1.0]}'
```

Validate steering sign with zero speed, then speed commands 1, 2, and 3.
Do not begin with speed 8 or 10.

After a non-undervoltage VESC fault has cleared and voltage has remained above
the recovery threshold:

```bash
ros2 service call /vehicle/clear_motor_fault std_srvs/srv/Trigger '{}'
```
