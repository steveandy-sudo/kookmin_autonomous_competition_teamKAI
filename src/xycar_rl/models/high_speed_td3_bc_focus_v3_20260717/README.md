# High-speed TD3+BC focus v3

Selected checkpoint: `camera_speed_td3_bc_epoch_042.pth`

The model was resumed from the full epoch-30 TD3+BC state. Training used only
transitions whose stored action produced the stored next state. The difficult
16-20 m curve expert transitions were sampled five times more often.

Gazebo fixed-start gate, seeds `20260724..20260728`:

```text
speed cap          7.0
lap success        5/5
mean speed command 6.94..6.97
max CTE            0.141..0.202 m
large oscillation  1 event in 1/5 runs, 0 events in the other 4/5 runs
```

Cap 7.25 failed and was not approved. Cap 8 completed only 1/5 runs. These
limits are Gazebo results, not real-car approval.

## Gazebo full-track run

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

MODEL_DIR="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models/high_speed_td3_bc_focus_v3_20260717"

ros2 run xycar_rl rollout_policy \
  --project-root "$PWD" --policy-kind camera_speed_td3_bc \
  --checkpoint "$MODEL_DIR/camera_speed_td3_bc_epoch_042.pth" \
  --episodes 1 --max-steps 3000 --seed 20260724 \
  --start-progress-fraction 0.0 --recovery-probability 0.0 \
  --s-curve-focus-probability 0.0 --action-noise 0.0 \
  --min-speed-command 4 --max-speed-command 12 \
  --speed-cap-command 7.0 --device cuda
```

Start `rl_sim.launch.py` in another terminal before this command.

## Real-car shadow

Every real-car test starts at cap 4 regardless of the Gazebo cap or epoch.

```bash
MODEL_DIR="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models/high_speed_td3_bc_focus_v3_20260717"

ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:="$MODEL_DIR/camera_speed_td3_bc_epoch_042.pth" \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```

Do not set `drive_enabled=true` until shadow output, steering sign, inference
latency, stale-sensor stop, a physical emergency stop, and a wheels-up test all
pass.
