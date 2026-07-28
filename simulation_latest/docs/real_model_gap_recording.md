# Real Model Gap Recording

`xycar_run_recorder` records one comparable real-vehicle session per driving
model. It does not publish motor commands.

The default bag contains:

- original MJPEG camera frames and PNG-compressed canonical model input
- IMU, odometry, TF, and camera calibration
- applied motor command, policy output/debug/status, and rule diagnostics
- VESC voltage, currents, input power, ERPM, converted m/s, duty,
  temperature, and fault code
- vehicle-PC CPU, memory, temperature, and free disk space

Uncompressed `sensor_msgs/Image` topics are never selected. The session also
stores the model SHA256, Git state, ROS parameters, topic list, process list,
bag information, and average topic rates.

## Record

Start the camera, IMU, and motor bridge first. Perception and the driving node
may be started after recording begins.

Camera:

```bash
cd /home/xytron/kookmin_ty
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch app_wide_camera_calib wide_camera_rectified.launch.py \
  balance:=0.3
```

SparkFun Razor IMU:

```bash
cd /home/xytron/kookmin_ty/simulation_latest
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch xycar_imu xycar_imu.launch.py
```

LR-ASPP canonical perception:

```bash
cd /home/xytron/kookmin_ty/simulation_latest
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch lane_seg_control \
  lane_seg_lraspp_low_latency_real.launch.py \
  cpu_threads:=4 \
  max_output_rate_hz:=7.0
```

Start the desktop `모터 구동` icon before recording VESC telemetry.

Recorder:

```bash
cd /home/xytron/kookmin_ty/simulation_latest
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

MODEL="$PWD/xycar_ws/src/xycar_rl/models/high_speed_td3_bc_dual_dagger_v6_20260717/camera_speed_td3_bc_epoch_051.pth"

ros2 run xycar_run_recorder record_model_run \
  --session dual_dagger_e51_run01 \
  --driver rl \
  --model-path "$MODEL"
```

The recorder starts without `/xycar_motor`. It keeps discovering the deferred
motor, RL, IL, and rule topics and records them if a driving node is started
later. A session with no driving node remains a valid sensor-only bag.

By default, the compressor subscribes only to the canonical model input. This
keeps optional BEV and mask publishers inactive in the low-latency perception
node. For a perception diagnostic session, add
`--include-perception-intermediates`; do not use that option when measuring
control latency.

Press `Ctrl+C` once to stop. File-level zstd finalization can take several
seconds. Output is written below `~/rosbags/model_gap/`.

Use a unique session name for every model and repeat each condition at least
three times. Keep camera position, exposure, track placement, battery state,
speed cap, and target offset unchanged between models.

Before recording, verify that every required stream is producing messages:

```bash
ros2 topic hz /wide_camera_mjpeg/image_raw/compressed
ros2 topic hz /imu
ros2 topic hz /perception/canonical_road_image
ros2 topic echo /vehicle/vesc_state --once
```

The recorder's preflight also checks message arrival. It refuses to start if
the compressed camera, IMU, or VESC telemetry is missing. A motor command is
not required.

For a rule-based reference:

```bash
ros2 run xycar_run_recorder record_model_run \
  --session rule_reference_run01 \
  --driver rule \
  --model-label canonical_stanley_pursuit
```

For the three-mode hybrid driver:

```bash
ros2 run xycar_run_recorder record_model_run \
  --session hybrid_run01 \
  --driver hybrid \
  --model-label avg17_model_curve_rule_hwj_cone \
  --notes "caps=10,target_left_offset=0.10"
```

The recorder includes `/hybrid/mode`, `/hybrid/debug`,
`/hybrid/cone_clusters`, and `/hybrid/cone_path` when they appear.

Use `--allow-missing` only when the missing sensor is intentional and record
the reason with `--notes`.

Each session contains `topic_rates.csv`, `manifest.json`, `rosbag_info.txt`,
the exact rosbag command, parameter dumps, USB/process snapshots, and the
zstd-compressed bag. `/xycar_motor` is the requested steering and speed. The
current vehicle does not expose a measured front-wheel angle; `/odom` is used
as an independent measured linear speed when available. VESC telemetry stores
both raw ERPM and m/s converted using the recorded `speed_to_erpm` parameters.
