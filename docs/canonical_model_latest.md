# Latest Canonical BEV Policy

Validated on 2026-07-15 after collection, training and held-out evaluation.

- model: `drive_canonical_policy_scripted.pt`
- SHA-256: `dd8cb6c2ccfca5a438b08e90f930f50527a88b5169cae9e8239dd129f78dbb21`
- collected rows: `30000`
- general-drive rows: `21547`
- recovery rows: `8453`
- recovery ratio: `0.28177`
- stopped rows: `0`
- discarded stopped pre-roll rows: `433`
- real-camera-like canonical artifact events: `1041`
- best epoch: `50`
- validation MAE (Xycar angle command): `3.83424`
- held-out rows: `5000`
- held-out test MAE: `3.37915`
- held-out recovery MAE: `4.28435`
- held-out RMSE: `5.51879`

## Simulation

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
colcon build --packages-select kaiev26_msgs xycar_perception \
  xycar_gazebo_bridge il_data_tools --symlink-install
source install/setup.bash

ros2 launch il_data_tools sim_policy_drive.launch.py \
  drive_enabled:=true device:=cuda
```

The launch defaults to the packaged canonical model and
`/perception/canonical_road_image`.

## Real Car Shadow

```bash
MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"
sha256sum "$MODEL"

ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false use_compressed_image:=false \
  scan_topic:=/scan drive_enabled:=false device:=cpu
```

Do not enable the motor until the canonical image, steering sign, sensor
synchronization and timeout stop have passed the root README checklist.
