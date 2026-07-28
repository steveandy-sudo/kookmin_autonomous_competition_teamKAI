# Real lane segmentation and canonical perception

This document records the real-car perception work completed on 2026-07-22.
It covers perception only. None of the launches in `lane_seg_control` publishes
motor commands.

## Why this path exists

The original real-car pipeline detects white boundaries and a yellow
centerline inside `xycar_perception`, applies the calibrated homography, and
normalizes the result to the simulation-compatible canonical image. Field
testing exposed three additional needs:

1. compare multiple segmentation runtimes without changing the BEV contract;
2. preserve valid YOLO white detections that were removed by downstream
   thickness, geometry, or tracker filters;
3. turn fragmented white masks into at most one stable line per physical side
   while leaving the observed yellow dashes unchanged.

`lane_seg_control` implements that alternate frontend while retaining the
existing canonical topic and color contract.

## Pipeline contract

```text
/wide_camera/rect/image_raw
  -> YOLO ONNX or LR-ASPP MobileNetV3-Small
  -> /lane_seg/white_boundary_mask
  -> /lane_seg/yellow_centerline_mask
  -> calibrated 640x480 BEV
  -> /lane_seg_bev/debug_image
  -> 256x144 canonical conversion
  -> optional canonical white-line fitter
  -> /perception/canonical_road_image
```

Canonical output uses BGR `(36,36,36)` for background, `(255,255,255)` for
white boundaries, and `(0,220,255)` for the yellow centerline. The BEV remains
unchanged by the canonical white-line fitter.

## White-line fitting rules

- Sliding windows collect row-wise centers from white-mask fragments.
- Every accepted center on the same physical side contributes to one fit;
  sparse fragments are connected over their full visible span instead of only
  between neighboring windows.
- Each side produces at most one white line, symmetrically for left and right.
- When yellow marks exist, a robust full-height yellow divider is built only
  for left/right classification. The published yellow mask remains fragmented
  exactly as detected.
- White pixels left of the divider can only feed the left fit and pixels right
  of it can only feed the right fit.
- When yellow is absent, all visible white fragments are treated as one lane
  and assigned left or right from their global horizontal position. Opposite
  sides are never joined.
- A quadratic fit is allowed only with enough consistent sections; otherwise
  the fitter uses a linear model to avoid unsupported curvature.

The detailed windows, accepted centers, fit order, and RMSE are published on
`/perception/canonical_white_fit_debug`.

## Existing perception improvements

The `xycar_perception` path also includes the measured 2026-07-21 real-camera
homography with independent y coordinates for all four source corners. This
compensates camera roll instead of forcing the top and bottom source points
onto common rows. A six-pixel lateral validity margin retains masks crossing a
calibrated BEV side edge. For the YOLO backend, white-mask preservation can
bypass morphology, component thickness, geometry, and tracker replacement so
an accepted white detection reaches canonical output.

## Models

| Model | Location | SHA256 |
| --- | --- | --- |
| YOLO `best_512.onnx` | `lane_seg_control/models` | `cfb87b0d4ff54917728ea2bcfbbd9eca31c5de030652949a452195ac279f5629` |
| LR-ASPP MobileNetV3-Small `256x144` | `lane_seg_control/models` | `45ab4744f5edee1464441f15681e0b77ab94415a1193558b113c2bd619120fe8` |
| YOLO11n `512` candidate | `xycar_perception/models` | `198583b23e3791994ba53dd81485a5744781d89232ee68850fdd8b5dbd88ebdd` |
| YOLO11n `256` candidate | `xycar_perception/models` | `907057eb3a3c7b27dea14d0fcb9bbef75cdb8c6859ef12de8c7714887f827b85` |
| YOLO26n `256` candidate | `xycar_perception/models` | `54362594f074cb91badb660f56e55808844cb00b28f886ddadb92ae8354c8fce` |

The 256 ONNX files are stored only in `xycar_perception/models`; duplicate
local copies under `lane_seg_control` are intentionally ignored.

## Build

```bash
cd /home/xytron/kookmin_ty/simulation_latest
set +u
source /opt/ros/humble/setup.bash

colcon build --packages-up-to lane_seg_control xycar_rule_drive \
  --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE
```

## Perception-only execution

Run the rectified camera separately, then choose one frontend.

YOLO ONNX with canonical output and no path generation:

```bash
ros2 launch lane_seg_control lane_seg_canonical_only.launch.py
```

LR-ASPP MobileNetV3-Small with canonical white fitting:

```bash
ros2 launch lane_seg_control lane_seg_lraspp_canonical_only.launch.py
```

Inspect the source, BEV, masks, fitted canonical, and fitting diagnostics:

```bash
rviz2 -d "$(ros2 pkg prefix --share xycar_rule_drive)/rviz/real_yolo_camera_canonical.rviz"
```

Measure the output rate without starting a driving node:

```bash
ros2 topic hz /perception/canonical_road_image
```

## Rosbag verification

The exporter samples compressed-camera data at 10 Hz and produces a
four-panel image containing the rectified source, raw BEV before canonical,
fitted canonical image, and sliding-window debug view. Its header includes the
nearest recorded steering/speed command and the Focus-v3 epoch-42 policy's raw
prediction. Recorded commands are read from storage and never republished.

```bash
ros2 run lane_seg_control export_lraspp_bag_montages \
  /home/xytron/rosbags/full_vehicle/canonical_compressed_run_01_20260716_134859 \
  --sample-hz 10
```

## Safety boundary

These perception launches do not publish `/xycar_motor`. Keep camera and
perception validation separate from the policy process. Before real driving,
verify canonical geometry and rate in RViz, run the selected policy in shadow
mode, confirm steering sign, and only then enable motor output.
