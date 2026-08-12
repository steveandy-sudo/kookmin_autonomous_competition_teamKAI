# Xycar integrated rule drive

This package contains only the real-car integrated drive path:

1. the lane RULE candidate,
2. cone slalom override,
3. YOLO plus LiDAR vehicle avoidance override,
4. final mode arbitration and the space-bar safety gate.

It intentionally does not install SLAM, localization, Nav2, map, or waypoint
navigation nodes. In rule-only mode the authority order is:
`CONE_RULE > YOLO_LIDAR_AVOIDANCE > RULE`.

Build from the workspace root:

```bash
colcon build --symlink-install --packages-up-to \
  wide_camera xycar_vesc_driver my_rule_msgs my_rule xycar_map_nav
```

Run the real-car rule-only stack from the source tree:

```bash
./src/xycar_map_nav/scripts/run_complete_rule_only.sh
```

Run only the read-only integrated RViz visualization while a driving stack or
bag replay is already publishing topics:

```bash
ros2 launch my_rule drive_visualization_rviz.launch.py
```

For a bag replayed with `--clock`, add `use_sim_time:=true`.  The view contains
the rectified camera, lane-model overlay, object-YOLO box overlay, LiDAR cone
clusters, cone-evidence regions, and separate connected lane/cone waypoint
lines. Canonical-road and planned-path image panels are intentionally omitted.
It subscribes to existing outputs and publishes visualization markers only.

The complete real-car launch can start the same view with:

```bash
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  drive_enabled:=false enable_rviz:=true
```

## Shared camera input for bag replay and real driving

Build once after a source update:

```bash
cd /home/subin/kookmin_autonomous_competition_teamKAI/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select lane_seg_control my_rule xycar_map_nav
source install/setup.bash
```

The wide-camera JPEG stream is decoded in exactly one process.  To prevent a
large raw DDS image from becoming a new bottleneck, it publishes two tailored
depth-1 streams: `/wide_camera/lane_rect/image_raw` at 512x288 and
20 Hz, and `/wide_camera/object_rect/image_raw` at 640x512 and 5 Hz.  The lane
stream is first rectified through the existing 1024x576 map and then reduced to
the model's 512x288 input, while the object stream retains the camera's 5:4
aspect ratio.  Thus JPEG decoding happens once without changing either model's
intended rectification geometry.
The larger object stream uses reliable QoS so fragmented raw frames are not
silently dropped; the lane stream remains best-effort to prioritize freshness.

### Review the recorded outputs on this bag

Use four terminals.  In every new terminal, run:

```bash
cd /home/subin/kookmin_autonomous_competition_teamKAI/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
BAG_PATH='/home/subin/kookmin_autonomous_competition_teamKAI/8.11 오전주행-20260811T095714Z-1-001/8.11 오전주행/bag'
```

Terminal 1 safely replays the camera, LiDAR, and recorded perception outputs.
The waypoint/path topics in this command are the results saved in the bag, not
results recalculated with the current source parameters.  The whitelist
intentionally excludes `/xycar_motor`:

```bash
ros2 bag play "$BAG_PATH" --clock 40 --rate 0.5 --start-paused \
  --topics \
  /wide_camera_mjpeg/image_raw/compressed \
  /scan \
  /rule_drive/connected_yellow_path \
  /rule_drive/diagnostics \
  /my_rule/cone_path \
  /my_rule/cone_clusters
```

Terminal 2 is the only camera decompression/rectification process:

```bash
ros2 launch my_rule compressed_camera_republish.launch.py \
  use_sim_time:=true
```

Terminal 3 visualizes the recorded lane/cone waypoints and camera without
loading Torch or Ultralytics:

```bash
ros2 launch my_rule drive_visualization_rviz.launch.py use_sim_time:=true
```

Terminal 4 shows playback position, total duration, percentage, effective
rate, and PLAYING/PAUSED/DONE state:

```bash
ros2 run my_rule bag_progress "$BAG_PATH"
```

Return to Terminal 1 and press Space to begin.  The Humble player controls are:

- `Space`: pause/resume
- `Right arrow`: publish one next message while paused
- `Up arrow`: increase playback rate by 10 percent
- `Down arrow`: decrease playback rate by 10 percent
- `Ctrl+C`: stop playback

Set the initial rate with `--rate 0.25`, `--rate 0.5`, `--rate 1.0`, or
`--rate 2.0`.  Start at a bag-time offset with `--start-offset 28.0`.  This bag
is 99.098 seconds long, and the progress monitor reports offsets against that
full duration even when playback starts in the middle.

The bag does not contain either object-YOLO debug-image topic.  Therefore the
RViz panel named `YOLO Object Detection (live model)` is expected to say
`No Image` in recorded-output review.  RViz intentionally omits the Canonical
Road and Planned Lane Path Image panels; the rectified camera and recorded
waypoint lines remain visible.

### Rerun the current models from the bag

On a computer whose ROS Python runtime already has Torch and Ultralytics, play
only the two raw sensor topics in Terminal 1:

```bash
ros2 bag play "$BAG_PATH" --clock 40 --rate 0.5 --start-paused \
  --topics /wide_camera_mjpeg/image_raw/compressed /scan
```

Run the same decompressor command in Terminal 2.  In Terminal 3, point the two
models at their dedicated shared-decoder outputs and disable their internal
rectifiers:

```bash
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  use_sim_time:=true drive_enabled:=false enable_rviz:=true \
  camera_image_topic:=/wide_camera/lane_rect/image_raw \
  object_camera_image_topic:=/wide_camera/object_rect/image_raw \
  camera_use_compressed_image:=false \
  camera_enable_rectify:=false \
  direct_model_rectify_enabled:=false
```

On this computer, Torch and Ultralytics are in `teamkai-yolo`.  Use the
optional model-only Python prefix below.  The model nodes consume the shared
raw BGR topics without loading `cv_bridge` inside conda, so no conda package or
NumPy version is changed:

```bash
source /home/subin/anaconda3/etc/profile.d/conda.sh
conda activate teamkai-yolo
source /opt/ros/humble/setup.bash
source /home/subin/kookmin_autonomous_competition_teamKAI/xycar_ws/install/setup.bash

ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  use_sim_time:=true drive_enabled:=false enable_rviz:=true \
  model_python_prefix:=/home/subin/anaconda3/envs/teamkai-yolo/bin/python \
  camera_image_topic:=/wide_camera/lane_rect/image_raw \
  object_camera_image_topic:=/wide_camera/object_rect/image_raw \
  camera_use_compressed_image:=false \
  camera_enable_rectify:=false \
  direct_model_rectify_enabled:=false
```

This raw-sensor-only replay is the mode that generates paths from the current
source and current cone/lane parameters.

In the top-down RViz map, vehicle forward (+X in `laser_frame`) points toward
the top of the screen. The light orange sector is the LiDAR scan candidate
range; the darker left/right sectors are the cone-group seed regions. Their
angles and ranges come from `cone_path_visualizer.yaml`, synchronized with
`cone_control.yaml`.

### Real camera and LiDAR driving on the vehicle computer

Start the actual wide-camera and LiDAR drivers first.  Their topics must be
`/wide_camera_mjpeg/image_raw/compressed` and `/scan`.  Run the same dedicated
decoder in its own terminal, without simulation time:

```bash
ros2 launch my_rule compressed_camera_republish.launch.py \
  use_sim_time:=false
```

Start the real drive stack in a separate terminal using the same shared-camera
arguments.  On the machine with `teamkai-yolo`, prepare only this model/drive
terminal as follows.  Only the two model processes use conda Python; the ROS
drivers and shared camera decoder retain the system ROS Python:

```bash
source /home/subin/anaconda3/etc/profile.d/conda.sh
conda activate teamkai-yolo
source /opt/ros/humble/setup.bash
source /home/subin/kookmin_autonomous_competition_teamKAI/xycar_ws/install/setup.bash

ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  use_sim_time:=false drive_enabled:=false enable_rviz:=true \
  model_python_prefix:=/home/subin/anaconda3/envs/teamkai-yolo/bin/python \
  camera_image_topic:=/wide_camera/lane_rect/image_raw \
  object_camera_image_topic:=/wide_camera/object_rect/image_raw \
  camera_use_compressed_image:=false \
  camera_enable_rectify:=false \
  direct_model_rectify_enabled:=false
```

Only after the camera, lane path, YOLO detections, LiDAR, and steering command
are confirmed should the vehicle operator rerun the last command with
`drive_enabled:=true`.  Do not set `use_sim_time:=true` during real driving.
Keep the target computer's existing Python/conda environment; these commands
do not install packages or modify that environment.

After the 2026-08-06 BEV zero-point calibration, the default static lane target
is 9 cm left of the perceived yellow line. This is the conservative starting
value that preserves the previously validated physical trajectory.
Dynamic avoidance offsets are added only while the avoidance state is active.
