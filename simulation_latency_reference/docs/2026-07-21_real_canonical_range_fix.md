# 2026-07-21 real canonical range diagnosis and fix

## Symptom

The live vehicle produced a canonical image that appeared to cover only about
0.6m. White and yellow lines were also bent or removed after being visible in
the YOLO and BEV debug images.

The camera calibration, YOLO checkpoint, launch parameters, and clean commit
were verified on the vehicle. Reprocessing the same MJPEG timestamp with clean
`6bee30c` reproduced the runtime output exactly, so the issue was downstream of
camera rectification and YOLO inference.

## Metric-coordinate clarification

The canonical contract remains 256x144 over 0.0-1.5m forward. The physical
camera sees the floor between approximately 0.5m and 1.5m:

```text
canonical row 0:   1.5m
canonical row 48:  1.0m
canonical row 96:  0.5m
canonical row 144: 0.0m
```

The 0.0-0.5m region is intentionally invalid because the sensor housing and
vehicle body occlude it. A single component's row span is its detected length,
not the camera's complete forward range. Therefore `component_span * 1.5/144`
must not be used alone to decide whether the 1.5m coordinate contract failed.

Changing `dst_bottom_y_ratio` from 2/3 to 1.0 without changing the physical
mapping would stretch the measured 0.5-1.5m interval to 0.0-1.5m and inflate
all forward distances by 50 percent. That change was deliberately not made.

## Actual defects fixed

1. The BEV validity image previously warped the entire rectified camera image.
   Pixels outside the calibrated source trapezoid could therefore enter the
   nominally invisible near field. The validity image is now intersected with
   the source trapezoid before the homography.
2. Sparse YOLO white pixels are robustly fitted and redrawn as a continuous
   boundary. This removes the dotted white-mask regression and gives live and
   rosbag processing the same normalized representation. YOLO yellow dashes
   remain current-frame observations.
3. A short yellow dash was rejected when it did not overlap both white
   components in the forward direction. A non-overlapping boundary is now
   treated as unavailable evidence rather than contradictory evidence.
4. YOLO yellow masks no longer pass through the color-mask geometry filter a
   second time. The topology tracker still rejects a yellow candidate that is
   physically outside an observed white boundary.
5. When both white boundaries are temporarily absent, a YOLO yellow component
   in the central 20-80 percent of the canonical width is retained. An outside
   yellow wall remains rejected.
6. The normal ASUS launch now defaults to raw compressed MJPEG with repository
   rectification enabled. This is the same input path used during rosbag
   replay; an external rectified topic must be requested explicitly.

## Added diagnostic topics

```text
/perception/canonical_pregeometry_white_mask
/perception/canonical_pregeometry_yellow_mask
/perception/canonical_pretrack_white_mask
/perception/canonical_pretrack_yellow_mask
/perception/canonical_valid_mask
/perception/canonical_metric_debug
```

`canonical_metric_debug` overlays 0.5m, 1.0m, and 1.5m guides. Its dark red
lower region is outside the calibrated camera view. The normal model input is
still `/perception/canonical_road_image`; the guide image is diagnostic only.

## Local validation

The perception test suite passed 58 tests. An eight-second competition-bag
sample after the final filtering change produced:

```text
frames:                         81
pre-geometry yellow present:   97.5%
pre-tracker yellow present:    97.5%
final yellow present:          97.5%
geometry full yellow drops:    0
tracker full yellow drops:     0
white geometry full drops:     0
white tracker full drops:      0
valid canonical rows:          0-93
```

The exact percentages depend on the replayed bag interval. The important
result is that an observed YOLO yellow mask was not completely removed by a
later stage in this validation window.

## Vehicle-side verification

Do not start a motor or policy node during this check.

```bash
cd ~/kookmin_ty
git switch simulation
git pull --ff-only origin simulation

source /opt/ros/humble/setup.bash
colcon build --packages-up-to xycar_perception --symlink-install
source install/setup.bash

ros2 launch xycar_perception \
  real_yolo_canonical_from_raw_asus.launch.py
```

Open the metric and stage views in separate terminals:

```bash
ros2 run rqt_image_view rqt_image_view \
  /perception/canonical_metric_debug

ros2 run rqt_image_view rqt_image_view \
  /perception/canonical_pretrack_yellow_mask

ros2 run rqt_image_view rqt_image_view \
  /perception/canonical_road_image
```

Record one stationary straight-road sample:

```bash
./scripts/record_real_camera_diagnostic.sh straight_range_fix 15
```

If the physical 0.5m, 1.0m, and 1.5m marks do not align with the metric debug
guides, the remaining issue is camera extrinsic geometry: mount height, pitch,
yaw, or the source trapezoid. Matching intrinsic calibration files does not
prove that the camera is mounted at the same pose. Recalibrate the source
trapezoid from measured floor marks rather than changing the metric scale.
