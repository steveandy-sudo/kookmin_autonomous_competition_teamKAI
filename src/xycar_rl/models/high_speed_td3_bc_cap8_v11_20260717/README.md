# High-speed TD3+BC milestones

Every checkpoint is actor-only and intended for shadow evaluation first.
Every real-car test starts at cap 4 regardless of epoch.
The staged cap is for Gazebo testing and is not a real-car approval.

```bash
MODEL_DIR="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models/high_speed_td3_bc_cap8_v11_20260717"
```

## Epoch 072

Offline candidate only; not approved for deployment.

Initial real shadow cap: `4.0`

```bash
ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:=$MODEL_DIR/camera_speed_td3_bc_epoch_072.pth \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```

## Epoch 075

Gazebo cap `8.0`: seed `20260724` completed, seed `20260726` off track at `19.49m`.

This checkpoint is not approved. Use it only as the next DAgger learner.

Initial real shadow cap: `4.0`

```bash
ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:=$MODEL_DIR/camera_speed_td3_bc_epoch_075.pth \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```
