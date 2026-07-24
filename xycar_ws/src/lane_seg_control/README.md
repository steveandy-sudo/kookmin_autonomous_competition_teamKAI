# lane_seg_control

KookminTY real-car and Gazebo lane segmentation pipeline. The current runtime
uses the packaged LR-ASPP MobileNetV3-Small TorchScript model; the legacy YOLO
model is not part of the active driving path.

Pipeline:

```text
/wide_camera_mjpeg/image_raw/compressed
  -> decode + rectify only the newest selected 7 Hz frame
  -> LR-ASPP MobileNetV3-Small (RGB 256x144)
  -> in-process white/yellow masks
  -> in-process calibrated BEV (640x660, 1.4m x 1.5m)
  -> /perception/canonical_road_image (256x144 bgr8)
```

The real-car low-latency path does not serialize three intermediate images,
run an approximate-time synchronizer, or warp a color BEV. The legacy
two-node adapter remains available for pixel-equivalence checks.

Canonical-only conversion keeps the calibrated `640x480` BEV unchanged and
extends its canvas to `640x660`. The additional rows retain accepted near-field
segmentation masks that the same homography projects below `y=479`. The full
`x=0..639`, `y=0..659` result is converted to `256x144` without another crop.

The output colors remain compatible with the simulation-trained policies:

- background: BGR `(36, 36, 36)`
- white boundary: BGR `(255, 255, 255)`
- yellow centerline: BGR `(0, 220, 255)`

Build:

```bash
cd /home/xytron/kookmin_ty/simulation_latest
source /opt/ros/humble/setup.bash
colcon build --packages-up-to lane_seg_control --symlink-install
source install/setup.bash
```

Run after the real MJPEG camera is publishing:

```bash
ros2 launch lane_seg_control lane_seg_lraspp_low_latency_real.launch.py
```

Use the Gazebo-matched camera and BEV profile in simulation:

```bash
ros2 launch lane_seg_control lane_seg_lraspp_sim_canonical.launch.py
```

The simulation profile consumes `/image_raw`, publishes debug images at 7Hz,
and widens only the simulated BEV destination trapezoid so its 0.80m white-line
spacing matches the real rosbag canonical input. Real-camera homography values
remain unchanged.

## LR-ASPP MobileNetV3-Small

The packaged TorchScript semantic-segmentation model keeps the same mask, BEV,
and canonical topic contract on the vehicle and in Gazebo:

The generic launch also defaults to the direct in-process canonical path:

```bash
ros2 launch lane_seg_control lane_seg_lraspp_canonical_only.launch.py \
  image_topic:=/wide_camera/rect/image_raw
```

It consumes RGB ImageNet-normalized `256x144` input and maps output classes as
`0=background`, `1=white boundary`, and `2=yellow centerline`. The inference
node still publishes full camera-resolution masks so the calibrated BEV
homography and the `256x144` canonical output remain unchanged.

After canonical conversion, the LR-ASPP launch fits only the left and right
white masks with bottom-up sliding windows. Every accepted same-side center
contributes to the linear fit; a quadratic is allowed only when at least eight
tracked sections provide consistent curvature evidence.
When canonical yellow marks exist, their row centers are robustly fitted to a
single straight `x(y)` divider and extended across the full canonical height.
White pixels left and right of that divider feed only the matching lane fit.
When no yellow pixels exist, all white fragments are treated as one lane and
assigned left or right from their global horizontal position.
Each side produces at most one white curve. Sparse same-side fragments use
their complete row-wise center axis when adjacent sliding-window centers are
too close, so the single fitted line extends to the top of the canonical image.
This rule is symmetric for left and right lanes; fragments never join across
the yellow divider.
The fitted divider is internal only: canonical output retains the original
fragmented yellow mask, and the BEV output remains unchanged. The final fitted
model input is `/perception/canonical_road_image`; windows, sampled centers,
fit order, and RMSE are visible on
`/perception/canonical_white_fit_debug`. Disable this stage with
`canonical_white_fit_enabled:=false`.

Open the LR-ASPP input, unchanged BEV, fitted canonical, and sliding-window
debug topics in RViz:

```bash
rviz2 -d "$(ros2 pkg prefix --share xycar_rule_drive)/rviz/real_yolo_camera_canonical.rviz"
```

For a legacy adapter comparison, use
`direct_canonical_enabled:=false publish_intermediate_topics:=true`.
The live default keeps queue depth 1 to prioritize the newest frame.

`/lane_seg/diagnostics` contains model, total callback, decode/rectify,
canonical, source-age, replaced-input, and stale-input timings. Use
`scripts/measure_live_pipeline_latency.py` for a 30-second p50/p95 report.

Export 10 Hz side-by-side montages directly from a compressed-camera rosbag:

```bash
ros2 run lane_seg_control export_lraspp_bag_montages \
  /path/to/rosbag \
  --sample-hz 10
```

Each JPEG contains the rectified source, unchanged BEV before canonical,
fitted canonical road image, and canonical sliding-window debug image from the
same camera timestamp. Its header shows the nearest recorded `/xycar_motor`
steering and speed commands plus the Focus-v3 epoch-42 policy's raw steering
and speed prediction for that fitted canonical frame. Recorded commands are
read directly from the bag by storage timestamp; they are never replayed or
published. The policy value is the raw actor output before runtime filtering.

This package does not publish `/xycar_motor`, `/cmd_vel`, or legacy lane motor
commands. A driving policy should consume `/perception/canonical_road_image`
in a separate process after shadow validation.

## Packaged models

The two runtime models required by this package are versioned with the source:

| File | Runtime | SHA256 |
| --- | --- | --- |
| `models/best_512.onnx` | YOLO segmentation, `512x512` | `cfb87b0d4ff54917728ea2bcfbbd9eca31c5de030652949a452195ac279f5629` |
| `models/kookmin_lane_lraspp_mbv3s_256x144.pt` | LR-ASPP MobileNetV3-Small TorchScript | `45ab4744f5edee1464441f15681e0b77ab94415a1193558b113c2bd619120fe8` |

The `kookmin_lane_yolo11n_256` and `kookmin_lane_yolo26n_256` candidates are
owned by `xycar_perception/models`; local copies under this package are test
artifacts and are intentionally not duplicated in Git.
