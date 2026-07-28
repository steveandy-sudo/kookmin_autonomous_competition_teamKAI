# High-speed TD3+BC milestones

Every checkpoint is actor-only and intended for shadow evaluation first.
Every real-car test starts at cap 4 regardless of epoch.
The staged cap is for Gazebo testing and is not a real-car approval.

```bash
MODEL_DIR="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models/high_speed_td3_bc_dual_dagger_v6_20260717"
```

## Epoch 051

Validated Gazebo cap: `7.5` (`5/5`, seeds `20260724..20260728`)

Initial real shadow cap: `4.0`

```bash
ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:=$MODEL_DIR/camera_speed_td3_bc_epoch_051.pth \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```

## Epoch 054

Gazebo cap `7.5`: `4/5`; not selected.

Initial real shadow cap: `4.0`

```bash
ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:=$MODEL_DIR/camera_speed_td3_bc_epoch_054.pth \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```
