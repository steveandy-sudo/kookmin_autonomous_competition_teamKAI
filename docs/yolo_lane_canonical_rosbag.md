# YOLO Lane Mask to Canonical ROS Bag Pipeline

For ASUS installation, live-camera shadow testing, and the physical safety
gate, use [`real_vehicle_yolo_lane_20260720.md`](real_vehicle_yolo_lane_20260720.md).

## Pipeline

1. Decode `/wide_camera_mjpeg/image_raw/compressed`.
2. Rectify the fisheye image using the measured 1280x1024 calibration.
3. Run the packaged YOLO11n 512 segmentation model.
4. Merge class 0 instances as `white_boundary` and class 1 instances as
   `yellow_centerline`.
5. Warp both binary masks with the measured real-camera BEV homography.
6. Normalize the result to the shared 256x144 canonical image contract.

The canonical image uses BGR `(36, 36, 36)` for road, `(255, 255, 255)` for
white boundaries, and `(0, 220, 255)` for the yellow centerline. Lane width is
normalized to five pixels.

## Build

```bash
cd ~/kookmin_sim_to_real
source /opt/ros/humble/setup.bash
colcon build --packages-up-to xycar_rule_drive --symlink-install
source install/setup.bash
```

## Inspect a ROS Bag in RViz

```bash
ros2 launch xycar_rule_drive real_yolo_lane_bag_rviz.launch.py \
  bag_path:="/absolute/path/to/rosbag_directory"
```

The default playback rate is 0.25x so the GPU processes every recorded camera
frame. Increase `playback_rate` only after checking the canonical topic rate.

Optional overrides:

```bash
ros2 launch xycar_rule_drive real_yolo_lane_bag_rviz.launch.py \
  bag_path:="/absolute/path/to/rosbag_directory" \
  playback_rate:=0.5 \
  yolo_device:=0 \
  yolo_confidence:=0.25 \
  yolo_image_size:=512
```

## Output Topics

- `/perception/rectified_camera_image`: calibrated camera frame
- `/perception/yolo_debug_image`: YOLO masks and confidence overlay
- `/perception/canonical_road_image`: 256x144 imitation/RL model input
- `/perception/canonical_white_mask`: normalized white mask
- `/perception/canonical_yellow_mask`: normalized yellow mask
- `/perception/debug_image`: BEV lane and centerline debug image

The launch is shadow-only. It sends no command to the physical motor topic.

## Lightweight Vehicle Model

The launch files use the model installed with `xycar_perception`:

```text
share/xycar_perception/models/kookmin_lane_yolo11n_512.pt
```

Independent test-mask metrics:

- all classes: mAP50 0.880, mAP50-95 0.566
- white boundary: mAP50 0.981, recall 0.969
- yellow centerline: mAP50 0.779, recall 0.718

The original YOLO11s 640 model remains under `models/lane_segmentation` as an
accuracy reference. The 5.8 MB YOLO11n checkpoint is the real-vehicle default
because it was 2.1 times faster in the same PyTorch CPU benchmark.

## Ryzen 5 Vehicle PC

Build and benchmark the exact ASUS machine before enabling motor output:

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select kaiev26_msgs xycar_perception --symlink-install
source install/setup.bash

MODEL="$(ros2 pkg prefix xycar_perception)/share/xycar_perception/models/kookmin_lane_yolo11n_512.pt"
ros2 run xycar_perception benchmark_yolo_lane \
  --model "$MODEL" \
  --source /absolute/path/to/one_real_camera_frame.png \
  --device cpu \
  --image-size 512 \
  --cpu-threads 4 \
  --iterations 100
```

Run live perception through the same raw-MJPEG and internal-rectification path
used for rosbag replay:

```bash
ros2 launch xycar_perception real_yolo_canonical_asus.launch.py
```

To diagnose an externally rectified topic explicitly, override all three
transport settings together:

```bash
ros2 launch xycar_perception real_yolo_canonical_asus.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  use_compressed_image:=false \
  enable_rectify:=false
```

Measure the complete output rather than relying only on the model benchmark:

```bash
ros2 topic hz /perception/canonical_road_image
```

Use at least 15 Hz for initial low-speed driving. High-speed testing should
target at least 20 Hz with end-to-end latency below 80 ms. If the ASUS cannot
reach 15 Hz while the obstacle detector is running, test
`yolo_image_size:=416` as a fallback and re-check lane accuracy on rosbag data.
The current independent test dropped yellow-centerline recall from 0.718 at
512 to 0.613 at 416, so 416 is not the default.

## CPU Budget With Obstacle Detection

- Lane YOLO: PyTorch YOLO11n-seg, 512 input, four CPU threads, up to 15 Hz.
- Driving policy: existing compact camera-speed actor, about 1.2 MB.
- Obstacle YOLO: use a nano detection model at 416 input, two CPU threads, and
  5 Hz. Track detections between YOLO frames.
- Do not render `/perception/yolo_debug_image` during physical driving. The
  node now skips overlay rendering automatically when the topic has no
  subscribers.

Lane perception must keep priority. If both YOLO nodes cannot meet the target
latency, run obstacle inference only in obstacle-course sections or replace the
two independent networks with one jointly labelled multi-class model.
