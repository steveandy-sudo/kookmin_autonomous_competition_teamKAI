# Ubuntu Codex Prompt: Test Team K.A.I. Xycar ROS2 IL Data Tools

You are working on the Team K.A.I. Xycar ROS2 project for the 2026 Kookmin University Autonomous Driving Competition finals.

Goal:
Test and harden the `il_data_tools` ROS2 Humble package on Ubuntu 22.04 before running it on the real Xycar Y-model vehicle.

Important safety context:
- This package is for data collection only.
- The recorder must not publish to `/xycar_motor`.
- The real vehicle must have only one `/xycar_motor` publisher.
- Do not start, stop, or manage Docker automatically.
- Do not modify existing driving packages such as `track_drive`, `my_motor`, `xycar_msgs`, `xycar_cam`, `xycar_lidar`, or other provided packages unless explicitly asked.

Expected real vehicle workspace:

```bash
/home/xytron/xycar_ws
```

Expected package location:

```bash
/home/xytron/xycar_ws/src/il_data_tools
```

Target environment:
- Ubuntu 22.04
- ROS2 Humble
- Python package style: `ament_python`
- Xycar motor command topic: `/xycar_motor`
- Motor message: usually `xycar_msgs/msg/XycarMotor`
- Some legacy/example code may use `std_msgs/msg/Float32MultiArray`; keep `motor_msg_type` configurable.

Current intended dataset format:

```text
~/xycar_ws/datasets/il/YYYYMMDD_HHMMSS_session_name/
  samples.csv
  images/front/*.jpg
  scan/*.npz
  debug/
  metadata.json              # optional, can be disabled
  README_session.md          # disabled by default
```

Default saved sample data:
- `timestamp_ns`
- front image path
- LiDAR scan path
- motor steering angle
- motor speed, saved for filtering/analysis even if speed is rule-based during driving
- mission label
- source mode
- notes

LiDAR details:
- The Xycar front distance sensor is a 2D LiDAR.
- Range is approximately 0.12 m to 12 m.
- The hardware sample rate is listed as 5,000 Hz, but ROS `/scan` publication rate may be much lower, often around 5-15 Hz.
- Each saved `scan/*.npz` should contain:
  - `ranges`
  - `intensities`
  - `angle_min`
  - `angle_max`
  - `angle_increment`
  - `range_min`
  - `range_max`

Recommended first checks:

```bash
cd /home/xytron/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select il_data_tools
source install/setup.bash
ros2 pkg executables il_data_tools
```

Expected executables:

```text
il_data_tools dataset_recorder_node
il_data_tools mission_labeler_node
```

Run static/script checks if practical:

```bash
python3 -m compileall src/il_data_tools
python3 src/il_data_tools/scripts/summarize_dataset.py --help
python3 src/il_data_tools/scripts/bag_to_dataset.py --help
bash -n src/il_data_tools/scripts/check_topics.sh
bash -n src/il_data_tools/scripts/record_bag.sh
```

Run topic preflight:

```bash
bash ~/xycar_ws/src/il_data_tools/scripts/check_topics.sh
```

Important: If `/xycar_motor` has more than one publisher, do not drive.

Test mission labeler:

```bash
ros2 run il_data_tools mission_labeler_node
```

Keys:

```text
0 idle
1 lane_drive
2 cone_drive
3 hill_drive
4 pedestrian_avoid
5 vehicle_follow
6 vehicle_overtake
7 traffic_light_start
8 route_select
9 shortcut
p parking
r recovery
x bad_data
q quit
```

Dry-run recorder command:

```bash
ros2 launch il_data_tools record_dataset.launch.py \
  session_name:=dry_run \
  save_scan_npz:=true \
  save_side_images:=false \
  write_metadata:=false \
  write_session_readme:=false
```

If real sensors are not available, create a simulated ROS2 test that publishes:
- `sensor_msgs/msg/Image` on `/usb_cam/image_raw/front`
- `sensor_msgs/msg/LaserScan` on `/scan`
- `xycar_msgs/msg/XycarMotor` on `/xycar_motor`, or use `motor_msg_type:=std_msgs/Float32MultiArray` and publish `std_msgs/msg/Float32MultiArray`
- `std_msgs/msg/String` on `/il/mission_label`

Then verify the recorder creates:

```text
samples.csv
images/front/*.jpg
scan/*.npz
```

Verify `samples.csv` rows include valid paths and labels:

```bash
python3 ~/xycar_ws/src/il_data_tools/scripts/summarize_dataset.py \
  ~/xycar_ws/datasets/il/<SESSION_DIR>
```

Potential issues to check and fix:
- `cv_bridge` import errors
- `xycar_msgs` import/build dependency problems
- `rosbag2_py` import errors in `bag_to_dataset.py`
- Launch parameter type conversion problems
- Empty `scan_path` if `/scan` timestamps are outside sync tolerance
- Recorder should still never publish `/xycar_motor`
- Metadata and README files should be optional; `README_session.md` should stay off by default

After fixing anything, run:

```bash
cd /home/xytron/xycar_ws
colcon build --symlink-install --packages-select il_data_tools
source install/setup.bash
ros2 pkg executables il_data_tools
```

Final response requested:
- Summarize whether `colcon build` passed.
- Summarize whether dry-run recording created `samples.csv`, front images, and LiDAR npz files.
- Report the exact dataset session directory.
- Report any missing ROS dependencies.
- Confirm no `/xycar_motor` publisher is created by the recorder.
