# ASUS Real-Vehicle YOLO Lane Perception

Date: 2026-07-20
Updated: 2026-07-21 (real-camera calibration recovery)
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
- Immutable measured camera calibration:
  `xycar_perception/config/wide_camera_fisheye_1280x1024_20260708.yaml`
- Perception launch: `real_yolo_canonical_asus.launch.py`
- Raw-camera recovery launch: `real_yolo_canonical_from_raw_asus.launch.py`
- Perception plus policy shadow launch:
  `real_yolo_camera_speed_shadow.launch.py`
- CPU benchmark: `ros2 run xycar_perception benchmark_yolo_lane`
- Runtime check: `scripts/check_asus_yolo_runtime.sh`
- Driver calibration restore: `scripts/install_real_camera_calibration.sh`

Model SHA-256:

```text
7cd02f180ce5e5f3d7066e634ea17b16418e02dc0c2e93465b3bfb3bf2a5c9fd
```

Camera calibration measured on 2026-07-08:

```text
resolution: 1280x1024
model: equidistant fisheye
fx: 725.9760698863265
fy: 725.9655676685525
cx: 685.4283617571
cy: 497.80435337486205
D: [-0.023278262724104614, 0.0040440926898205444,
    -0.006251740223863257, 0.00246793717714305]
balance: 0.3
RMS reprojection error: 0.45765194028955725 px
SHA-256: 0bc9b224e8105b7c9da2f280097d5f6ea75395482f0c504d288745cbd5a25dfe
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

### Restore a changed real-camera calibration

The repository's perception node now defaults to the dated immutable profile
above. To restore the same values in the real vehicle's
`app_wide_camera_calib` package, stop the camera nodes and run:

```bash
cd ~/kookmin_sim_to_real
git switch simulation
git pull --ff-only

./scripts/install_real_camera_calibration.sh ~/xycar_ws

cd ~/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select app_wide_camera_calib --symlink-install
source install/setup.bash
```

The script preserves the previous YAML beside it as a timestamped `backup`
file. Restart the vehicle camera launch after rebuilding. Do not rectify an
already rectified topic a second time.

If `/wide_camera/rect/image_raw` still looks wrong, bypass that output and let
this repository rectify the unmodified MJPEG frame using the packaged values:

```bash
ros2 launch xycar_perception real_yolo_canonical_from_raw_asus.launch.py
```

This recovery launch reads
`/wide_camera_mjpeg/image_raw/compressed`, sets `enable_rectify=true` exactly
once, and publishes the corrected view on
`/perception/rectified_camera_image`. For policy shadow using the same bypass:

```bash
ros2 launch xycar_rl real_yolo_camera_speed_shadow.launch.py \
  image_topic:=/wide_camera_mjpeg/image_raw/compressed \
  use_compressed_image:=true enable_rectify:=true \
  drive_enabled:=false deployment_speed_cap:=3.0
```

Compare `/perception/rectified_camera_image` before enabling motor output. If
this image is still geometrically wrong, the camera mount or lens has changed
and the old intrinsic calibration must not be forced; recalibrate the physical
camera instead.

### Diagnose a live-versus-rosbag canonical mismatch

A valid comparison must use the same source frame and preserve all intermediate
images. On the real vehicle, stop driving, place the car on a representative
straight section, start exactly one perception launch, and capture 15 seconds:

```bash
cd ~/kookmin_sim_to_real
./scripts/record_real_camera_diagnostic.sh straight_center 15
```

The output is written below `~/kookmin_camera_diagnostics/`. Move the complete
timestamped directory to the simulation PC. It contains:

- raw compressed camera and driver-rectified camera topics
- the rectified frame actually consumed by perception
- YOLO overlay, BEV debug, canonical image, and white/yellow masks
- the full `/xycar_camera_perception` parameter dump
- Git commit, model checksum, calibration checksum, topic types, and publishers

Interpret the replay in this order:

1. Check that the Git commit and both checksums match this repository.
2. Check that only one `/xycar_camera_perception` and one camera rectifier ran.
3. Check whether the launch consumed raw MJPEG with `enable_rectify=true`, or
   rectified `sensor_msgs/Image` with `enable_rectify=false`.
4. Replay only the raw input through the repository's recovery launch. If its
   new canonical output differs from the canonical saved in the same bag, the
   real vehicle used different code or parameters.
5. If replay reproduces the saved canonical but differs from an older bag, the
   source image, camera pose, focus, exposure, or physical lens has changed.

Replay the raw camera without replaying the previously generated perception
topics:

```bash
# Terminal 1
ros2 launch xycar_perception real_yolo_canonical_from_raw_asus.launch.py \
  use_sim_time:=true

# Terminal 2
ros2 bag play /path/to/diagnostic_bag --clock --topics \
  /wide_camera_mjpeg/image_raw/compressed
```

The most useful physical reference capture has the car stationary and centered,
both white boundaries visible, the yellow dash visible, and 0.5m transverse
marks in view. Also record one left curve and one right curve after the straight
capture. Do not change the camera mount between captures.

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
