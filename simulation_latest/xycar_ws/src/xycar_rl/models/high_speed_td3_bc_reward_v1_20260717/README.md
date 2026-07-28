# High-speed TD3+BC milestones

Every checkpoint is actor-only and intended for shadow evaluation first.
All real-car tests start at cap 4 regardless of epoch.

Gazebo gate on seeds 20260724..20260728 selected **epoch 015**. It completed
5/5 laps at cap 6 with max CTE 0.141..0.179m and zero large-oscillation events.
Cap 7 completed 4/5 and cap 8 failed, so neither is approved for real testing.

```bash
MODEL_DIR="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models/high_speed_td3_bc_reward_v1_20260717"
```

| Epoch | SHA-256 |
|---:|---|
| 005 | `ac6e5285c9b248e3a6a53cc6d1ff07b563a0fb5348c019bf401f420a84e0d214` |
| 010 | `4753b88db7f44e3c20ae75efbbcc195ded1a95456fedac6e4cc965384261215b` |
| 015 | `79e0e3b47c090fed77e4478072b9ab61ba9b5e24c823044411655d6bc854bd28` |
| 020 | `706706e41851d9d6fe3bbfaee2e5a07b00f66b00270bf9d51601b11f2a0a66e5` |

## Epoch 005

Planned Gazebo cap: `4.0`

Initial real shadow cap: `4.0`

```bash
ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:=$MODEL_DIR/camera_speed_td3_bc_epoch_005.pth \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```

## Epoch 010

Planned Gazebo cap: `5.0`

Initial real shadow cap: `4.0`

```bash
ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:=$MODEL_DIR/camera_speed_td3_bc_epoch_010.pth \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```

## Epoch 015

Validated Gazebo cap: `6.0` (5/5 fixed-start seeds)

Selected candidate. Initial real shadow cap: `4.0`

```bash
ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:=$MODEL_DIR/camera_speed_td3_bc_epoch_015.pth \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```

## Epoch 020

Rejected Gazebo cap: `8.0` (off track)

Comparison only. Initial real shadow cap: `4.0`

```bash
ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:=$MODEL_DIR/camera_speed_td3_bc_epoch_020.pth \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```
