# ASUS Real-Vehicle YOLO Lane Perception

Date: 2026-07-20
Branch: `simulation`

## Purpose

The real camera has reflections, faded yellow paint, and exposure changes that
made the color-threshold lane masks differ from Gazebo. The new path segments
only learned lane pixels and then applies the existing measured camera
rectification, BEV homography, and canonical normalization.

```text
/wide_camera/rect/image_raw
  -> YOLO11n-seg 512 on CPU
  -> white_boundary / yellow_centerline masks
  -> measured BEV homography
  -> 256x144 canonical road image
  -> camera-only driving policy
```

The canonical contract is unchanged, so existing simulation-trained BC and
TD3+BC policies do not need a new input shape.

## Delivered Files

- Model: `xycar_perception/models/kookmin_lane_yolo11n_512.pt`
- Perception launch: `real_yolo_canonical_asus.launch.py`
- Perception plus policy shadow launch:
  `real_yolo_camera_speed_shadow.launch.py`
- CPU benchmark: `ros2 run xycar_perception benchmark_yolo_lane`
- Runtime check: `scripts/check_asus_yolo_runtime.sh`

Model SHA-256:

```text
7cd02f180ce5e5f3d7066e634ea17b16418e02dc0c2e93465b3bfb3bf2a5c9fd
```

## 1. Clone and Install

```bash
cd ~
git clone --branch simulation \
  git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  kookmin_sim_to_real
cd ~/kookmin_sim_to_real

source /opt/ros/humble/setup.bash
python3 -m pip install --user \
  -r xycar_ws/src/xycar_perception/requirements-real-yolo-cpu.txt

rosdep install --from-paths xycar_ws/src --ignore-src -r -y

colcon build --packages-up-to xycar_rl --symlink-install
source install/setup.bash
```

The requirements pin NumPy below 2 because ROS 2 Humble `cv_bridge` commonly
uses the NumPy 1.x ABI. Do not perform a broad system-wide package upgrade on
the vehicle PC immediately before testing.

## 2. Preflight and CPU Benchmark

Use one real rectified camera frame as the benchmark input:

```bash
./scripts/check_asus_yolo_runtime.sh \
  "$PWD" /absolute/path/to/real_camera_frame.png
```

This verifies Python imports, the packaged model checksum, and 100 production
wrapper inferences. Then measure the complete ROS pipeline because model-only
latency excludes camera transport, rectification, BEV, and ROS publication.

Initial acceptance targets:

- model benchmark: mean below 70ms and p95 below 100ms
- `/perception/canonical_road_image`: at least 15Hz
- high-speed candidate testing: at least 20Hz and end-to-end latency below 80ms

If the combined lane and obstacle networks cannot maintain 15Hz, keep lane
inference at 512 and reduce obstacle inference to 416 at 5Hz. Running the lane
model at 416 reduced yellow-centerline recall from 0.718 to 0.613 and is only an
emergency fallback.

## 3. Perception-Only Test

Start the real camera first and verify its exact type:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/kookmin_sim_to_real/install/setup.bash

ros2 topic info /wide_camera/rect/image_raw -v
ros2 topic hz /wide_camera/rect/image_raw
```

For the normal already-rectified `sensor_msgs/Image` topic:

```bash
ros2 launch xycar_perception real_yolo_canonical_asus.launch.py
```

For an unrectified compressed MJPEG source:

```bash
ros2 launch xycar_perception real_yolo_canonical_asus.launch.py \
  image_topic:=/wide_camera_mjpeg/image_raw/compressed \
  use_compressed_image:=true enable_rectify:=true
```

Observe rates and images in another terminal:

```bash
ros2 topic hz /perception/canonical_road_image
rviz2 -d "$(ros2 pkg prefix xycar_rule_drive)/share/xycar_rule_drive/rviz/real_lane_drive.rviz"
```

RViz subscribes to the YOLO overlay and therefore adds rendering work. Close
RViz before measuring production latency or driving.

Perception acceptance:

- white masks lie only on physical white boundaries
- yellow masks lie only on yellow centerline dashes
- no yellow mask appears outside a white road boundary
- output remains 256x144 with fixed canonical colors and five-pixel lines
- straight and curved rosbag sections both meet the rate target

## 4. Camera-Only Policy Shadow

This launch connects the YOLO canonical output to the approved camera-only
checkpoint. Its defaults are shadow-only and speed cap 3.

```bash
ros2 launch xycar_rl real_yolo_camera_speed_shadow.launch.py \
  drive_enabled:=false deployment_speed_cap:=3.0
```

Inspect without touching the motor:

```bash
ros2 topic echo /rl/policy_motor_shadow
ros2 topic echo /rl/policy_debug
ros2 topic echo /rl/policy_status
ros2 topic hz /rl/policy_motor_shadow
```

Shadow acceptance:

- output is continuous at about 15Hz
- left and right camera curves produce the physical steering sign
- steering stays within `-42..42` without repeated saturation
- speed never exceeds the configured deployment cap
- stopping the camera causes a zero command within 0.5 seconds
- no autonomous publisher writes `/xycar_motor`

## 5. Physical Test Gate

Do not enable driving from the first test. Use this order:

1. perception-only and rosbag review
2. policy shadow with wheels on the ground but motor output disabled
3. wheels raised, physical emergency stop held by an operator
4. straight 2-3m test at the lowest validated cap
5. one gentle curve
6. full temporary track
7. competition track only after the previous gates pass

Only after the gates pass:

```bash
ros2 launch xycar_rl real_yolo_camera_speed_shadow.launch.py \
  drive_enabled:=true deployment_speed_cap:=3.0
```

Verify exactly one publisher exists before moving:

```bash
ros2 topic info /xycar_motor -v
```

## 6. Running a Second Obstacle YOLO

The packaged lane model uses four CPU threads and is capped at 15Hz. Keep a
future obstacle detector small and asynchronous:

- YOLO nano detection model, not segmentation unless masks are required
- 416 input, two CPU threads, 5Hz
- track detections between inference frames
- publish objects independently from the lane callback
- lane perception and policy remain higher scheduling priority

The obstacle model is not included yet because obstacle labels and a trained
checkpoint have not been supplied. Do not run an unvalidated generic detector
in the motor-control path.
