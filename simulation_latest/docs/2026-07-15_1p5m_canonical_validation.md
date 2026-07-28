# 1.5m canonical BEV validation

## Why 1.5m

The old 1.0m BEV often showed only one 0.30m yellow dash. A 2.0m trial showed
three or four components when partial dashes at the image edges were included.
The selected model contract is therefore:

```text
image:          256x144 BGR
lateral range:  1.4m
forward range:  1.5m from the camera lens
white spacing:  0.824m between marking centers
dash pattern:   0.30m yellow + 0.30m gap
```

The raw BEV is `640x220`. Its physical guides are row 147 at 0.5m, row 73 at
1.0m, and row 0 at 1.5m. The lower region that the camera cannot see is removed
by a source-derived validity mask rather than by interpreting black pixels as
road markings.

## Calibration inputs

Real-camera geometry was measured from:

```text
/home/as/Downloads/same_location_20260714_232655
topic: /wide_camera_mjpeg/image_raw/compressed
frames: 435
resolution: 1280x1024
rate: 29.706Hz
```

The scene has transverse tape every 0.5m from the camera lens and no cones, so
cone detections cannot contaminate the geometry result. Competition-bag cone
sections are not used for this calibration.

Simulation geometry uses the SDF's known straight-road values: 0.30m dashes,
0.30m gaps, 0.824m white-center spacing, and the spawn pose 0.215m to the right
of the yellow line.

## Runtime comparison

Both results were captured through the ROS perception node after build, not
only through an offline image transform:

```text
                    real bag        Gazebo
yellow components   2               3
yellow centers      0.85, 1.40m     0.37, 0.93, 1.41m
white spacing       0.820m          0.826m
```

The different component count is the intended two-to-three range and comes
from the vehicle's phase within the 0.60m dash period. The white-spacing error
against the 0.824m target is 4mm in the sampled real frame and 2mm in Gazebo.

The active real and simulation profiles use observation-only canonical input.
They keep current-frame geometry filtering and washed-out-yellow recovery, but
disable missing-boundary synthesis, temporal coasting, and persistent output.

## Shadow preview

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch xycar_rule_drive real_lane_drive_1p5m_preview_rviz.launch.py \
  image_topic:=/wide_camera_mjpeg/image_raw/compressed \
  use_compressed_image:=true \
  enable_rectify:=true
```

This launch keeps `drive_enabled=false` and publishes commands only to the
shadow topic.
