# Ubuntu Codex Verification Prompt

아래 프롬프트를 Ubuntu 22.04 / ROS2 Humble 환경의 Codex에 그대로 넣어 주세요.

```text
You are working on Team K.A.I.'s ROS2 Humble Xycar project for the 2026 Kookmin University Autonomous Driving Competition finals.

Please answer and report in Korean.

Current branch should be:

  data_set

My role is imitation learning data collection, training, and evaluation.
Another teammate is responsible for rule-based mission logic, traffic light, pedestrian stop, speed planning, emergency stop, parking, and final /xycar_motor publication.

Important safety boundaries:
- Do not modify autonomous driving behavior in track_drive.
- Do not implement rule-based mission logic.
- Do not publish /xycar_motor from any imitation-learning tool.
- Do not create any ROS node that controls the vehicle.
- The learned models output steering only.
- Speed and all safety decisions remain rule-based.

The repository should contain a sibling ROS2 package:

  il_data_tools/

Your task is verification first, not feature work.
Only make minimal fixes if a verification step fails.

============================================================
1. Confirm file structure
============================================================

Run:

  cd ~/xycar_ws/src
  find . -maxdepth 4 -type f | grep -E "il_data_tools|build_drive|build_cone|build_overtake|mission_labeler|recorder|train_policy|eval_policy|benchmark_policy|README"

Expected important files:

  il_data_tools/package.xml
  il_data_tools/setup.py
  il_data_tools/il_data_tools/common_recorder_node.py
  il_data_tools/il_data_tools/mission_labeler_node.py
  il_data_tools/launch/record_drive_dataset.launch.py
  il_data_tools/launch/record_cone_dataset.launch.py
  il_data_tools/launch/record_overtake_dataset.launch.py
  il_data_tools/scripts/check_topics.sh
  il_data_tools/scripts/build_drive_dataset.py
  il_data_tools/scripts/build_cone_dataset.py
  il_data_tools/scripts/build_overtake_dataset.py
  il_data_tools/scripts/policy_models.py
  il_data_tools/scripts/policy_dataset.py
  il_data_tools/scripts/train_policy.py
  il_data_tools/scripts/train_drive_policy.py
  il_data_tools/scripts/train_cone_policy.py
  il_data_tools/scripts/train_overtake_policy.py
  il_data_tools/scripts/eval_policy.py
  il_data_tools/scripts/visualize_policy_predictions.py
  il_data_tools/scripts/export_policy_torchscript.py
  il_data_tools/scripts/export_policy_onnx.py
  il_data_tools/scripts/benchmark_policy_runtime.py
  il_data_tools/scripts/compare_models.py
  il_data_tools/README.md

If il_data_tools is missing, stop and report.

============================================================
2. Build verification
============================================================

Run:

  cd ~/xycar_ws
  colcon build --symlink-install --packages-select il_data_tools
  source install/setup.bash
  ros2 pkg executables il_data_tools

Expected executables:

  il_common_recorder
  il_mission_labeler

Also confirm installed scripts are runnable if possible:

  ros2 run il_data_tools check_topics.sh --help

If this fails because shell scripts do not support --help, that is okay.
But the scripts should be installed under the package.

============================================================
3. Safety check: /xycar_motor must not be published
============================================================

Search:

  grep -R "create_publisher.*XycarMotor\|create_publisher.*xycar_motor\|/xycar_motor\|xycar_motor" -n \
    ~/xycar_ws/src/il_data_tools \
    ~/xycar_ws/src/track_drive 2>/dev/null

Interpretation:
- il_data_tools may subscribe to /xycar_motor.
- il_data_tools may record /xycar_motor in rosbag scripts.
- il_data_tools must NOT have create_publisher(XycarMotor, ...).
- il_data_tools must NOT publish /xycar_motor.
- mission_labeler may publish /il/mission_label only.

Report clearly whether this safety check passes.

============================================================
4. Mission labeler smoke test
============================================================

Terminal 1:

  cd ~/xycar_ws
  source install/setup.bash
  ros2 run il_data_tools il_mission_labeler

Terminal 2:

  source ~/xycar_ws/install/setup.bash
  ros2 topic echo /il/mission_label

Press keys in Terminal 1:

  1 -> general_drive
  6 -> cone_drive
  7 -> vehicle_overtake
  b -> bad_data
  q -> quit

Expected:
- /il/mission_label publishes std_msgs/String at about 5 Hz.
- Label changes when keys are pressed.

If running interactive terminal is not possible, inspect the code and report that manual test is pending.

============================================================
5. Recorder dry-run
============================================================

Run:

  cd ~/xycar_ws
  source install/setup.bash
  ros2 launch il_data_tools record_drive_dataset.launch.py session_name:=dry_run

Expected:
- Node starts.
- Sensor-topic warnings are okay if the car/sensors are not running.
- Node must not crash immediately.
- It should create a session directory under:

  ~/xycar_ws/datasets/il/drive/

Check:

  find ~/xycar_ws/datasets/il/drive -maxdepth 2 -type f | tail -50

Expected session files:

  metadata.json
  samples.csv
  README_session.md

If no camera/motor/label topics exist, samples.csv may contain only the header. That is okay for dry-run.

============================================================
6. Dataset builder dummy test
============================================================

Create a tiny fake dataset with three sessions per profile and simple placeholder image files.
The builder only needs image paths to exist for this test.

Use Python or shell, but keep it inside /tmp or ~/xycar_ws/tmp_verify.

Then run:

  python3 ~/xycar_ws/src/il_data_tools/scripts/build_drive_dataset.py \
    --dataset-root /tmp/il_verify/il/drive \
    --output-dir /tmp/il_verify/processed/drive \
    --max-steer-deg 100 \
    --min-abs-speed 1.0 \
    --val-ratio 0.2 \
    --test-ratio 0.2

  python3 ~/xycar_ws/src/il_data_tools/scripts/build_cone_dataset.py \
    --dataset-root /tmp/il_verify/il/cone \
    --output-dir /tmp/il_verify/processed/cone \
    --max-steer-deg 100 \
    --min-abs-speed 1.0 \
    --val-ratio 0.2 \
    --test-ratio 0.2

  python3 ~/xycar_ws/src/il_data_tools/scripts/build_overtake_dataset.py \
    --dataset-root /tmp/il_verify/il/overtake \
    --output-dir /tmp/il_verify/processed/overtake \
    --max-steer-deg 100 \
    --default-duration-sec 4.0 \
    --min-abs-speed 1.0 \
    --val-ratio 0.2 \
    --test-ratio 0.2

Expected outputs:

  train.csv
  val.csv
  test.csv
  dataset_report.json

Confirm:
- drive CSV has columns:
  image_path, steer_norm, angle_deg, speed, mission_label, session_id, timestamp_ns
- cone CSV also preserves scan_npz_path.
- overtake CSV has phase and scan_npz_path.
- overtake phase goes from 0.0 to 1.0 between overtake_start and overtake_end.
- train/val/test are split by session, not random frames.

============================================================
7. Training script smoke checks
============================================================

Do not start long training yet.
Only check CLI and imports:

  python3 ~/xycar_ws/src/il_data_tools/scripts/train_policy.py --help
  python3 ~/xycar_ws/src/il_data_tools/scripts/train_drive_policy.py --help
  python3 ~/xycar_ws/src/il_data_tools/scripts/train_cone_policy.py --help
  python3 ~/xycar_ws/src/il_data_tools/scripts/train_overtake_policy.py --help
  python3 ~/xycar_ws/src/il_data_tools/scripts/eval_policy.py --help
  python3 ~/xycar_ws/src/il_data_tools/scripts/benchmark_policy_runtime.py --help
  python3 ~/xycar_ws/src/il_data_tools/scripts/compare_models.py --help

If PyTorch/torchvision/OpenCV are missing, report which dependency is missing.
Do not install dependencies unless I explicitly approve.

============================================================
8. Report format
============================================================

Please report in Korean with this table:

| Check | Result | Notes |
|---|---|---|
| File structure | PASS/WARN/FAIL | ... |
| colcon build | PASS/WARN/FAIL | ... |
| /xycar_motor safety | PASS/WARN/FAIL | ... |
| mission_labeler | PASS/WARN/FAIL/PENDING | ... |
| recorder dry-run | PASS/WARN/FAIL/PENDING | ... |
| dataset builder dummy test | PASS/WARN/FAIL | ... |
| training script CLI | PASS/WARN/FAIL | ... |

If you make fixes, list changed files and why.

Do not push to GitHub unless I explicitly ask.
```

