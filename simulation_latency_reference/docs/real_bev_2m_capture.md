# Real-camera 2m BEV calibration capture

Use this procedure only after the camera has been fixed in its final competition
mount. Moving the camera after capture invalidates the homography.

## Data layout

This site is used only for geometric calibration. Its illumination is not
representative of the competition venue, so do not tune white/yellow color
thresholds or reflection rejection from these bags.

Record these stationary scenes for 15 seconds each:

1. `center_on_yellow`: vehicle longitudinal axis parallel to the straight road,
   with the vehicle center on the yellow centerline.
2. `lane_center`: vehicle in its normal driving position between yellow and the
   right white boundary.
3. `left_offset_10cm`: same heading, shifted 10cm left from `lane_center`.
4. `right_offset_10cm`: same heading, shifted 10cm right from `lane_center`.
Then record `slow_straight` for 20 seconds while driving straight at command 3.
Do not include cones in these calibration bags. Four stationary bags plus the
slow straight bag are sufficient.

The current calibration course already has transverse white marks every `0.5m`
from the camera lens. Keep those marks unchanged and make sure the `0.5m`,
`1.0m`, `1.5m`, and `2.0m` marks are all visible. They intentionally look like
stop lines and are used only to fit the forward metric scale from the raw
camera image; they must not be used as longitudinal lane detections.

The existing vehicle sheet places the camera lens at `x=-0.04m`, `y=0.00m`,
`z=0.17m` from the front-axle center. Confirm that the final mount still has
those values. If unchanged, a mark `d` metres forward from the lens is
`d-0.04m` forward from `base_footprint`. This conversion is required when the
debug horizontal rows are labelled in vehicle coordinates.

Also write down the final camera lens height and its forward, left, and upward
offsets from the front-axle center. A photo of the tape measure and camera mount
is useful evidence.

## Prepare the vehicle

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

ros2 topic list -t | grep -E 'wide_camera|image_raw|camera_info'
ros2 topic hz /wide_camera_mjpeg/image_raw/compressed
```

The preferred source is the original compressed camera topic. Rectification
and all candidate BEV homographies can then be reproduced offline. Do not
change HSV, BEV, or canonical parameters before recording.

## Record each scene

From the repository root:

```bash
chmod +x scripts/record_real_bev_calibration.sh

./scripts/record_real_bev_calibration.sh center_on_yellow 15
./scripts/record_real_bev_calibration.sh lane_center 15
./scripts/record_real_bev_calibration.sh left_offset_10cm 15
./scripts/record_real_bev_calibration.sh right_offset_10cm 15
./scripts/record_real_bev_calibration.sh slow_straight 20
```

The script automatically chooses one active image topic, includes camera info
and `/tf_static` when available, stops cleanly after the requested duration,
and writes under:

```text
~/kookmin_bev_calibration_2m/
```

Each manifest records `distance_origin=camera_lens` and
`mark_interval_m=0.50`. Override them only if the physical marks change.

## Validate before transferring

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

python3 scripts/inspect_bev_calibration_bag.py \
  "$HOME/kookmin_bev_calibration_2m/YYYYMMDD_HHMMSS_center_on_yellow"
```

The expected result is:

- `1280x1024` images;
- approximately `30Hz`;
- at least 100 frames per stationary bag;
- a generated `bev_calibration_preview.png` showing the same fixed pose in its
  first, middle, and last frames.

Transfer the six complete bag directories, including `metadata.yaml`, database,
manifest, JSON report, and preview image. Do not transfer only the `.db3` file.

## Live shadow preview

After the raw bags are safely recorded, the selected 1.5m model input can be
viewed without motor output. The 2.0m marks remain useful for checking the
calibration even though the model view stops at 1.5m.

```bash
ros2 launch xycar_rule_drive real_lane_drive_1p5m_preview_rviz.launch.py \
  image_topic:=/wide_camera_mjpeg/image_raw/compressed \
  use_compressed_image:=true \
  enable_rectify:=true
```

The launch always includes `drive_enabled=false`. Do not use the preview to
move the physical vehicle.
