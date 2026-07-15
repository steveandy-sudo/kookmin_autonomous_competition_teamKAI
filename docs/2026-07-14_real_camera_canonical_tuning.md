# Real Camera Canonical Tuning - 2026-07-14

## Input data

The tuning was validated against the bags under `~/kookmin_ros_bag`, primarily:

- `fusion_01_20260714_144818_166158 (copy)`: 938 compressed camera frames
- `fusion_01_20260714_144924_330669 (copy)`: 3,493 compressed camera frames

The recorded camera contract is:

```text
topic: /wide_camera_mjpeg/image_raw/compressed
type: sensor_msgs/msg/CompressedImage
resolution: 1280x1024
distortion model: equidistant fisheye
```

These bags contain the unrectified compressed image, so bag review must use
`use_compressed_image:=true` and `enable_rectify:=true`.

## Root causes

1. Ceiling lights produce long, low-saturation reflections on the gray floor.
   HSV thresholding alone classifies some of them as white tape.
2. The real capture metadata specifies `forward_m_per_px: 0.005`, but the real
   perception profile used `0.010`. The canonical crop therefore enlarged only
   the nearest half of the real BEV.
3. The 1.2m canonical crop needs 240 source rows while the real BEV has 220.
   Padding the far 0.1m introduced a contrast edge that could become a white
   horizontal line.
4. Thick reflections were already filtered, but thin reflected streaks and
   checkerboard fragments could still pass the component filter.

## Applied correction

The simulation profile is unchanged. The real profile now:

- uses the measured `0.005 m/px` forward scale;
- widens the source homography horizontally to keep the opposite white
  boundary in view (`top=0.280..0.860`, `bottom=0.050..1.050`);
- ignores the padded far edge and the camera body region;
- rejects components that are too thick or occupy too much of the image;
- retains at most one coherent white boundary per side;
- retains only elongated, longitudinal yellow components;
- robustly fits each selected component with a quadratic curve and redraws it
  at the canonical 5px line width.

The output contract remains identical to simulation:

```text
size: 256x144
range: lateral 1.4m, forward 1.2m
background: BGR (36, 36, 36)
white lane: BGR (255, 255, 255)
yellow lane: BGR (0, 220, 255)
line width: 5px
```

## Temporal topology tracking

The real profile applies a stateful tracker after the per-frame canonical
mask extraction. It enforces the known lane topology and prevents a bright
reflection from immediately replacing a confirmed boundary:

```text
left white -- 0.412m -- yellow -- 0.412m -- right white
```

- a candidate closer than `0.18m` to yellow is rejected as an interior
  reflection;
- after an outer boundary is confirmed, a candidate more than `0.08m`
  inward from that track cannot replace it;
- a geometry-filtered lane candidate is accepted on its first frame;
- a physical boundary matching a synthesized width prediction replaces it
  after one frame;
- the search gate expands from `0.08m` to `0.25m` while a lane is missing;
- the last curve coasts for `0.30s` and remains searchable for `1.00s`;
- when one white boundary is missing, the other confirmed white curve is
  shifted continuously by the measured `0.824m` white-center spacing while
  yellow and the reference boundary remain valid;
- an interior candidate outside the `0.08m` width-prediction gate is ignored,
  so persistent lamp glare cannot replace the synthesized outer boundary;
- inferred lines do not change the raw road-segment observations.

The `/perception/canonical_tracking_debug` colors are:

```text
green       confirmed camera detection
orange      short coasting prediction
cyan        width-predicted missing white boundary
dark red    rejected white reflection candidate
purple      rejected yellow candidate
```

This synthesis affects only the canonical imitation-learning input and its
debug image. The raw road-segment observations remain unchanged.

The real BEV lateral scale is `0.002099m/px`, calibrated from 266 bag frames
where both white boundaries were visible. The previous value made the measured
lane only `109.37px` wide while width prediction used `150.67px`, producing a
`41.3px` mismatch. With the calibrated scale, measured and predicted white-line
center spacing both represent `0.824m`.

Checkerboard and transverse start-line frames are identified when white pixels
occupy at least 12% of the image width across six or more rows. Such a frame
does not update yellow or white tracks; the last longitudinal curves are held
until ordinary lane observations return.

After a valid road has been acquired, complete camera dropout no longer
produces an empty canonical image. The tracker coasts briefly and then enters
`persistent_predicted`: white boundaries retain their last fitted curvature,
the last yellow dash mask is retained, and a never-observed opposite boundary
can still be synthesized from the retained center/reference geometry. This
prediction has no time limit. A geometry-valid camera observation replaces it
on the first returning frame.

Yellow candidates are accepted only when they respect lane ordering:
`left white < yellow < right white`. With one white boundary, yellow must lie
on that boundary's road-interior side and remain within the configured
center-to-boundary range. A yellowish wall outside either white boundary is
rejected, and the previous center track remains predicted instead.

## Safe bag review

Build and run perception only. This does not start a motor publisher.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
colcon build --packages-select kaiev26_msgs xycar_perception \
  --symlink-install --allow-overriding kaiev26_msgs xycar_perception
source install/setup.bash

ros2 launch xycar_perception real_canonical_perception.launch.py \
  image_topic:=/wide_camera_mjpeg/image_raw/compressed \
  use_compressed_image:=true \
  enable_rectify:=true
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play \
  "$HOME/kookmin_ros_bag/fusion_01_20260714_144924_330669 (copy)" \
  --topics /wide_camera_mjpeg/image_raw/compressed
```

View the normalized model input and masks:

```bash
rqt_image_view /perception/canonical_road_image
rqt_image_view /perception/canonical_white_mask
rqt_image_view /perception/canonical_yellow_mask
```

To inspect the rectified driving camera, BEV detection, canonical model input,
lane markers, and optional masks together in RViz, use the shadow-mode launch:

```bash
ros2 launch xycar_rule_drive real_lane_drive_rviz.launch.py \
  image_topic:=/wide_camera_mjpeg/image_raw/compressed \
  use_compressed_image:=true \
  enable_rectify:=true
```

This launch publishes calculated commands only to `/xycar_motor_shadow` and
does not enable the physical motor topic. The RViz image panels use:

```text
/perception/rectified_camera_image  corrected driving camera
/perception/debug_image             BEV lane-detection debug
/perception/canonical_road_image    normalized model input
/perception/canonical_tracking_debug temporal/topology tracker debug
```

### Final 1.5m canonical view

The 50cm calibration marks showed that a 2.0m view contains three or four
center-dash components, while the old 1.0m view often contained only one.
The shared simulation/real contract therefore uses 1.5m, which keeps two or
three dashes depending on the vehicle phase. Use the shadow preview with:

```bash
ros2 launch xycar_rule_drive real_lane_drive_1p5m_preview_rviz.launch.py \
  image_topic:=/wide_camera_mjpeg/image_raw/compressed \
  use_compressed_image:=true \
  enable_rectify:=true
```

The calibrated geometry is:

```text
real source far row:           1.5m
real source near row:          0.5m
forward_m_per_px:              0.006818182
canonical_forward_range_m:     1.5
canonical image:               256x144, 1.4m wide
```

The real profile comes from the same-location bag with transverse tape at
0.5m intervals. The simulation profile comes from the known 0.30m dash,
0.30m gap, and 0.824m white-center spacing. Both produce parallel white lines
with a measured canonical spacing of about 0.826m.

Do not evaluate an older 1.0m, 1.2m, or 2.0m checkpoint as though it used this
contract. Retrain the model with the same 1.5m canonical input in simulation
and on the real vehicle.

## Validation result

On 495 processed frames from the 117-second bag:

```text
real white pixels p10/p50/p90:   0 / 427 / 678
real yellow pixels p10/p50/p90:  0 / 312 / 770
empty white ratio:  10.9%
empty yellow ratio: 18.6%
```

For comparison, 500 sampled simulation canonical frames were:

```text
sim white pixels p10/p50/p90:   1422 / 1552 / 1765
sim yellow pixels p10/p50/p90:   404 / 526 / 678
```

The lower real white occupancy is expected because the physical camera often
sees only one boundary. Broad reflection masks were removed. Before enabling
real driving, replay all bags in shadow mode and confirm steering sign and
sensor timeout behavior.

The temporal tracker was additionally replayed over all 3,493 camera frames
from the 117-second bag. Yellow was confirmed in 90.0% of frames and completely
empty canonical output was limited to 0.1%. White-boundary confirmation,
coasting, rejection, and inference transitions were all exercised. These
figures measure perception continuity, not closed-loop driving safety.
