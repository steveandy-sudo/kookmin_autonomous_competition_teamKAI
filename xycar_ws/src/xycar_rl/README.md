# xycar_rl

ROS 2 Humble reinforcement-learning package for the Kookmin Xycar track.

## Contracts

- Current state: canonical road image `3x90x160`; optional temporal actors stack
  the previous and current images as `6x90x160`. The steering-speed policy does
  not consume LiDAR and does not wait for `/scan`
- Current action: `[steering_norm, speed_norm]`; steering maps to `[-42, 42]`
  and speed maps to the range stored in each checkpoint (`4..10` legacy,
  `4..12` high-speed experiments)
- Legacy steering-only checkpoints remain supported for comparison
- Track reference: closed Catmull-Rom path extracted from the SDF yellow CAD dashes;
  the default path is reversed to match the real competition driving direction
- Default RL target: the yellow centerline itself (`0.0m` lateral offset)
- World pose: vehicle transform from `dynamic_pose/info`; odometry pose is only a
  fallback because reset/teleport can leave odometry coordinates stale
- Camera-only policies advance on each canonical image. Legacy policies still
  use the stamped camera/LiDAR join within `0.25s`.

The BC checkpoint was trained with `angle/100`. `ResNet18LidarActor` loads that
state dictionary strictly and applies `100/42` to preserve the same physical
steering command in the RL action space.

## Build

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
python3 -m pip install --user 'gymnasium>=0.29,<2'
colcon build --packages-up-to xycar_rl --symlink-install
source install/setup.bash
```

## Simulation and transitions

Start a paused Gazebo server. The Gym environment owns deterministic stepping.

```bash
ros2 launch xycar_rl rl_sim.launch.py headless:=-s enable_rviz:=false
```

For a visible Gazebo window, use `headless:=gui` instead of `headless:=-s`.

In a second terminal, record state-action-next-state transitions:

```bash
ros2 run xycar_rl rl_transition_recorder --ros-args \
  -p output_dir:=$PWD/datasets/rl/bc_rollout_01 \
  -p max_transitions:=100000
```

In a third terminal, run BC over normal and randomized S-curve recovery resets:

```bash
ros2 run xycar_rl rollout_policy \
  --project-root "$PWD" --policy-kind bc \
  --checkpoint "$PWD/models/il_policies/drive_canonical_real_reference_200k_20260715/drive_resnet18_lidar_best.pth" \
  --episodes 150 --max-steps 1200 --total-steps 100000 --device cuda
```

To watch one deterministic full lap from the competition start reference, set
`--episodes 1 --start-progress-fraction 0.0 --action-noise 0` and disable the
recovery and S-curve reset probabilities.

## Offline TD3+BC

```bash
ros2 run xycar_rl train_td3_bc \
  --transitions "$PWD/datasets/rl/bc_rollout_01" \
  --bc-checkpoint "$PWD/models/il_policies/drive_canonical_real_reference_200k_20260715/drive_resnet18_lidar_best.pth" \
  --output-dir "$PWD/models/rl/td3_bc_01" \
  --epochs 30 --batch-size 64 --num-workers 8 --device cuda
```

## Camera-only steering and learned speed

First distill a stable two-action policy from the validated steering rollout.
The `4` lower bound is the known full-lap speed, while `10` is the highest speed
covered by the current sim-to-real dynamics contract.

```bash
ros2 run xycar_rl train_camera_speed_bc \
  --transitions "$PWD/datasets/rl/yellow_centerline_20260716_140950" \
  --teacher-checkpoint "$PWD/models/rl/td3_bc_yellow_centerline_100k_20260716/td3_bc_best.pth" \
  --output-dir "$PWD/models/rl/camera_speed_bc_4to10" \
  --min-speed-command 4 --max-speed-command 10 --device cuda
```

Collect physical variable-speed transitions using that policy, then optimize
both actions with camera-only TD3+BC:

```bash
ros2 run xycar_rl rl_transition_recorder --ros-args \
  -p output_dir:=$PWD/datasets/rl/camera_speed_rollout_01 \
  -p max_transitions:=100000 -p require_action_trace:=true \
  -p require_lidar:=false

ros2 run xycar_rl rollout_policy \
  --project-root "$PWD" --policy-kind camera_speed_bc \
  --checkpoint "$PWD/models/rl/camera_speed_bc_4to10/camera_speed_bc_best.pth" \
  --min-speed-command 4 --max-speed-command 10 \
  --safe-speed-cap-command 5 --uncapped-speed-probability 0.35 \
  --speed-action-noise 0.08 --total-steps 100000 --device cuda

ros2 run xycar_rl train_camera_speed_td3_bc \
  --transitions "$PWD/datasets/rl/camera_speed_rollout_01" \
  --initial-checkpoint "$PWD/models/rl/camera_speed_bc_4to10/camera_speed_bc_best.pth" \
  --output-dir "$PWD/models/rl/camera_speed_td3_bc_4to10" \
  --min-speed-command 4 --max-speed-command 10 --device cuda
```

### High-speed expert and temporal DAgger

The measured-dynamics expert is simulation-only and never runs on the real car.
The repeatable fixed-start envelope on 2026-07-16 was `curve_min=8`, `max=12`:

```bash
ros2 run xycar_rl rollout_track_expert \
  --project-root "$PWD" --episodes 5 --max-steps 1400 \
  --min-speed-command 8 --max-speed-command 12 \
  --minimum-speed-curvature 0.9 --curvature-preview-m 2.0 \
  --start-progress-fraction 0.0 --recovery-probability 0.0
```

Result: 5/5 laps, mean speed command about `8.24`; `curve_min=8.5` fell to
4/5 and `curve_min=9` to 3/5. Command 12 maps to about `0.97m/s` in the current
simulation. Values above command 10 are extrapolated beyond the real-car speed
measurements and are not approved for real deployment.

The BC trainer supports curve weighting, two-frame temporal input, label latency
compensation, and a compact canonical-mask CNN:

```bash
ros2 run xycar_rl train_camera_speed_bc \
  --transitions "$PWD/datasets/rl/high_speed_expert_feedback_min7_max12_gui_20260716" \
  --output-dir "$PWD/models/rl/camera_speed_temporal_experiment" \
  --model-architecture compact --temporal-frames 2 \
  --label-lookahead-frames 1 \
  --min-speed-command 4 --max-speed-command 12 \
  --speed-target-source recorded --steering-weight-gain 4 \
  --straight-steering-weight 2 --straight-steering-threshold 0.12 \
  --max-abs-cross-track-error-m 0.30 \
  --max-abs-heading-error-deg 25 \
  --epochs 60 --device cuda
```

`label-lookahead-frames=1` pairs the current two-frame observation with the next
contiguous expert action. This compensates roughly one control frame of command
latency, so turn-in is learned before cross-track error changes sign. Episode
resets and dropped frames are never crossed while building this label.

The runtime applies an adaptive normalized-steering filter after inference:

- Straight: alpha `0.20`, rate limit `0.05`, deadband `0.02`
- Curve: alpha rises to `0.90`, rate limit rises to `0.38`
- The transition is continuous between normalized steering magnitudes `0.08`
  and `0.35`; small opposite corrections are taken through zero first
- A same-direction growing command receives `0.35` derivative lead above
  magnitude `0.04`; countersteer and straight noise do not receive this lead

The best 2026-07-17 candidate is
`models/rl/camera_speed_compact_temporal2_lead1_dagger_iter2_46k_20260717/camera_speed_bc_best.pth`.
It completed 4/5 neighboring fixed-start seeds with average speed commands near
`7.7..8.0` and peak commands near `10..10.7`. A targeted single-seed fine-tune
fixed that seed but regressed other seeds, so it is not the selected model. No
learned high-speed candidate has passed the required 5/5 gate yet, and an
isolated rerun after restarting Gazebo reproduced an off-track result. Keep all
of these checkpoints experimental and retain the real-car shadow cap of 4.

## Same-seed gate

```bash
ros2 run xycar_rl evaluate_closed_loop \
  --project-root "$PWD" \
  --bc-checkpoint "$PWD/models/il_policies/drive_canonical_real_reference_200k_20260715/drive_resnet18_lidar_best.pth" \
  --rl-checkpoint "$PWD/models/rl/td3_bc_01/td3_bc_best.pth" \
  --rl-kind td3_bc --episodes 20 --seed 20260716 \
  --output-dir "$PWD/analysis/rl_eval_td3_bc_01" --device cuda
```

## Residual online simulation

```bash
ros2 run xycar_rl train_residual_online \
  --project-root "$PWD" --base-kind scripted \
  --base-checkpoint "$PWD/models/rl/td3_bc_01/td3_bc_actor_scripted.pt" \
  --output-dir "$PWD/models/rl/residual_td3_01" \
  --total-steps 100000 --device cuda
```

## Real-car shadow

Run the real canonical perception first. Shadow is the default and cannot publish
motor commands until `drive_enabled:=true` is explicitly provided.

The launch default uses the tracked packaged BC TorchScript and therefore works
after a fresh clone. For a trained RL TorchScript, use `policy_kind:=scripted`.

```bash
ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_bc \
  checkpoint_path:=$PWD/models/rl/camera_speed_compact_temporal2_lead1_dagger_iter2_46k_20260717/camera_speed_bc_best.pth \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  drive_enabled:=false deployment_speed_cap:=4.0 \
  lidar_safety_enabled:=false steering_gain:=1.0 device:=cpu
```

Increase `deployment_speed_cap` only after shadow output and each closed-course
gate pass. The selected experimental policy may request up to `12`, but the
launch default keeps the first real-car run capped at `4`.

## 2026-07-16 validation status

- Training action range: speed command `4..10`; this is no longer limited to `7`.
- Camera-only transition data: `6,786` rows, exact action joins only, no scan files,
  `4` completed laps and `13` off-track terminals.
- Current operational candidate:
  `models/rl/camera_speed_bc_distilled_100k_v3_steering_priority_20260716/camera_speed_bc_best.pth`.
- Fixed competition-start test with `speed_cap=5`: 2/3 laps completed. This is an
  experimental simulation cap, not a real-car approval.
- The reward-trained TD3+BC candidates were less repeatable (at most 1/3 in the
  same gate), so they remain experiment artifacts rather than deployment defaults.
- Real-car first run remains shadow-only with `deployment_speed_cap=4`. Raise it
  one command at a time only after repeated closed-course passes.

See `docs/reinforcement_learning_roadmap.md` for gates and safety rules.
