# Xycar real-vehicle deployment

This profile runs the same camera perception, lane decision, and controller used
in Gazebo without starting any Gazebo or `ros_gz_bridge` process. The launch
defaults to shadow mode, where calculated commands are published only on
`/xycar_motor_shadow` and no `/xycar_motor` publisher is created.

## What the real vehicle must provide

- Ubuntu 22.04 and ROS 2 Humble
- `kaiev26_msgs`, `xycar_perception`, and `xycar_rule_drive` built in one workspace
- OpenCV, `cv_bridge`, `sensor_msgs`, and `std_msgs`
- One camera topic with either:
  - `sensor_msgs/msg/Image`, normally `/image_raw`; or
  - `sensor_msgs/msg/CompressedImage`, such as
    `/wide_camera_mjpeg/image_raw/compressed`
- The existing ROS1 motor container and ROS1-ROS2 dynamic bridge
- A motor subscriber accepting `std_msgs/msg/Float32MultiArray` as
  `[angle_command, speed_command]`
- A physical emergency-stop method and enough clear floor space

The lane controller does not require IMU data. Do not run
`xycar_gazebo_bridge`, `ros_gz_bridge`, or the Gazebo world on the vehicle.

## Files to copy into the vehicle workspace

Copy these package directories into the real vehicle's `xycar_ws/src`:

```text
kaiev26_msgs
xycar_perception
xycar_rule_drive
```

Build and source them:

```bash
source /opt/ros/humble/setup.bash
cd ~/xycar_ws
colcon build --packages-up-to kaiev26_msgs xycar_perception xycar_rule_drive \
  --symlink-install
source install/setup.bash
```

Use the vehicle's normal ROS domain before every run:

```bash
export ROS_DOMAIN_ID=7
```

## 1. Identify the live interfaces

Start the physical camera and the existing motor/bridge stack, then check:

```bash
ros2 topic list -t | grep -E 'image|xycar_motor'
ros2 topic info /image_raw -v
ros2 topic info /wide_camera_mjpeg/image_raw/compressed -v
ros2 topic info /xycar_motor -v
```

Required results:

- exactly one intended camera topic is active;
- its type is `Image` or `CompressedImage`;
- `/xycar_motor` has the real motor bridge as a subscriber;
- no other autonomous or keyboard node publishes motor commands.

If the motor topic is namespaced, pass it explicitly, for example
`motor_topic:=/xycar/xycar_motor`.

## 2. Shadow mode with a raw camera

Shadow is the default and cannot publish to the physical motor topic:

```bash
ros2 launch xycar_rule_drive real_lane_drive.launch.py \
  image_topic:=/image_raw \
  use_compressed_image:=false \
  drive_enabled:=false
```

Check the perception and calculated commands:

```bash
ros2 topic hz /perception/road_segments
ros2 topic echo /xycar_motor_shadow
rqt_image_view /perception/debug_image
```

Verify that:

- the white and yellow detections stay on the physical paint;
- the green target path remains between the yellow and right white lines;
- straight-road steering stays near zero;
- left and right steering signs match the physical vehicle convention;
- stopping the camera makes the shadow speed become zero within about 0.65 s.

## 3. Shadow mode with a compressed camera

Use this form when the real camera publishes MJPEG or another compressed image:

```bash
ros2 launch xycar_rule_drive real_lane_drive.launch.py \
  image_topic:=/wide_camera_mjpeg/image_raw/compressed \
  use_compressed_image:=true \
  drive_enabled:=false
```

The decoded image still appears as `/perception/debug_image` in raw ROS image
format.

## 4. Stationary motor check

Lift the drive wheels or otherwise secure the vehicle. Keep the launch in
shadow mode and compare `/xycar_motor_shadow` with small manual motor tests.
Confirm steering sign, steering center, motor direction, and that `Ctrl+C`
causes a zero command before enabling autonomous output.

The controller uses the measured real-car command-to-curvature table. Its
`+/-42` endpoints are still extrapolated from measurements up to command 30,
so the high-steering region needs another physical measurement when possible.

## 5. First autonomous run

Only after all shadow checks pass, start at speed command 1:

```bash
ros2 launch xycar_rule_drive real_lane_drive.launch.py \
  image_topic:=/image_raw \
  use_compressed_image:=false \
  motor_topic:=/xycar_motor \
  drive_enabled:=true \
  speed_command:=1.0
```

Keep a person at the emergency stop. Increase to the simulation default only
after low-speed straight and curve tests pass:

```bash
ros2 launch xycar_rule_drive real_lane_drive.launch.py \
  image_topic:=/image_raw \
  use_compressed_image:=false \
  motor_topic:=/xycar_motor \
  drive_enabled:=true \
  speed_command:=3.0
```

For a compressed camera, change the two camera arguments as shown in section 3.

## Parameters that must be validated on the physical track

The controller and ROS message contracts are shared with simulation, but these
vision parameters remain environment-dependent:

- exact camera topic, message type, resolution, and frame rate;
- exposure and white balance under the competition-room lighting;
- physical camera pitch and yaw;
- BEV source trapezoid (`src_*_ratio`);
- `lateral_m_per_px` and `forward_m_per_px`;
- HSV thresholds for white and yellow paint;
- lookahead distance and curve slowdown at real tire grip;
- command-to-curvature values above absolute command 30.

Edit the real profiles rather than the simulation profiles:

```text
xycar_perception/config/camera_perception_real.yaml
xycar_rule_drive/config/lane_rule_driver_real.yaml
```

## Stop and recovery

- Stop the launch with one `Ctrl+C`; the driver publishes a zero command during
  normal shutdown.
- If perception is missing or stale, the controller commands zero speed.
- A process kill, ROS failure, bridge failure, or hardware fault still requires
  the physical emergency stop and the real motor driver's own command timeout.
- Never run `keyboard_teleop`, another autonomous driver, and
  `real_lane_drive.launch.py drive_enabled:=true` at the same time.

## Behavioral-cloning policy

The repository also ships the trained camera+LiDAR TorchScript policy. It uses
the same physical `/xycar_motor` contract but has a separate shadow topic.

```bash
ros2 launch il_data_tools real_policy_inference.launch.py \
  image_topic:=/image_raw \
  scan_topic:=/scan \
  drive_enabled:=false

ros2 topic echo /il/policy_motor_shadow
ros2 topic echo /il/policy_debug
```

After stationary shadow validation, the conservative first drive is:

```bash
ros2 launch il_data_tools real_policy_inference.launch.py \
  image_topic:=/image_raw \
  scan_topic:=/scan \
  motor_topic:=/xycar_motor \
  drive_enabled:=true \
  speed_command:=1.0 \
  min_speed_command:=0.8
```

Do not run this drive mode at the same time as the rule-based driver. The
policy was trained in simulation, so successful simulation validation does not
remove the need for shadow, lifted-wheel, straight-line, and low-speed curve
tests on the physical vehicle.
