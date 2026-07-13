# hwj camera and lane code analysis

Source: `steveandy-sudo/kookmin_autonomous_competition_teamKAI`, branch `hwj`,
commit `681bd9fba52fc2776908c1ee241858621db55e63` inspected on 2026-07-13.

## Camera changes found

- `app_wide_camera`: MJPEG camera passthrough for the real camera.
- `app_wide_camera_calib`: 1280x1024 OpenCV fisheye calibration, rectification,
  LiDAR-image overlay, and static extrinsic TF tools.
- The intrinsic YAML in simulation already has the same numerical K/D values
  as the `hwj` file: RMS reprojection error `0.45765 px`, 100 calibration images.
- `my_rule_vehicle.launch.py` rectifies the real image with `balance=0.3` before
  feeding the lane code. Simulation now uses the same balance.
- Inverting the committed OpenCV fisheye K/D at the left and right image edges
  gives an effective full-image HFOV of `102.95 deg`; the nominal `170 deg`
  lens marketing value is therefore no longer used as the Gazebo image FOV.

The supplied screenshot shows a newer interactive overlay state than the YAML
committed under `hwj`: camera position in laser frame `(-0.105, 0, +0.090) m`
and optical correction `(roll, pitch, yaw)=(9, 0, 0) deg`. The translation is
applied to the Gazebo sensor poses. The roll is retained as overlay metadata
because it corrects the projection frame and is not evidence that the physical
camera image should be rolled by 9 degrees.

## Lane and center-path code found

- `my_rule/lane_node.py`: BEV conversion, white/yellow mask, Canny,
  `HoughLinesP`, direction-filtered line clustering, three-lane state tracking,
  center-curve correction, and Stanley steering.
- `my_rule/centerline_tracer_node.py`: selects YOLO `center_line` detections,
  masks occluding objects, extracts representative Hough segments inside each
  box, fits a degree-2 curve, and publishes it as `/center_curve`.
- `lane_bev_tools/scripts/dashed_center_path_test.py`: experimental direct
  dashed-line path extraction with sliding bands, degree-2 polynomial fitting,
  last-path fallback, and temporal smoothing.

Simulation keeps its camera-color-based KAIEV message contract instead of
requiring the real-car YOLO model. It adopts the reusable parts of the new
code: the measured fisheye rectification balance, dashed-path ROI, bottom
sensor-occlusion exclusion, quadratic center-path fitting, and temporal path
smoothing.
