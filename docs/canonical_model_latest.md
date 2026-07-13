# Latest Canonical BEV Policy

This file is generated only after collection, training, offline evaluation, and
GitHub publication all pass.

- model: `drive_canonical_policy_scripted.pt`
- SHA-256: `4826225e14300307d44fded47ee1c31b5f19833f80dc70263dae801fc8165699`
- collected rows: `50000`
- recovery rows: `14221`
- recovery ratio: `0.284`
- best epoch: `27`
- validation MAE (Xycar angle command): `1.7894374218419034`
- held-out test MAE (Xycar angle command): `2.69915643273569`

## Simulation

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
colcon build --packages-select kaiev26_msgs xycar_perception \
  xycar_gazebo_bridge il_data_tools --symlink-install
source install/setup.bash

MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"
ros2 launch il_data_tools sim_policy_drive.launch.py \
  model_path:="$MODEL" \
  image_topic:=/perception/canonical_road_image \
  drive_enabled:=true device:=cuda
```

## Real Car Shadow

```bash
MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"
ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  scan_topic:=/scan drive_enabled:=false device:=cpu
```

Only after the shadow steering sign and sensor timeout stop have been checked,
repeat the second command with `drive_enabled:=true speed_command:=3.0`.
