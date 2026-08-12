# Lap-time speed-only candidate

Date: 2026-07-23

This directory contains the actor-only checkpoint produced by freezing the
approved visual encoder and steering policy, including BatchNorm statistics,
and training only the range-expanded speed head.

## Model

```text
camera_speed_lap_time_speed_only_actor.pth
model type: camera_speed_temporal_resnet18_range_expanded
temporal frames: 2
trained speed command range: 4..100
SHA-256: 8f6673176631c70e6dbeac2d5830fa282e87f7b2825b36c16beeeee3f8ba70cf
```

The packaged file contains only the Actor state needed for inference. Critic
and optimizer states were intentionally removed.

## Gazebo validation

```text
5-lap screening: 5/5 complete, mean successful lap 16.188s
10-lap final gate: 8/10 complete, mean successful lap 16.295s
```

The 10-lap gate did not pass. This is the latest fast simulation candidate,
not a real-vehicle-approved final policy. Keep real-vehicle execution in shadow
mode and apply a low external speed cap.

Raw results are in `evaluation_5_laps.csv` and `evaluation_10_laps.csv`.

## Repeated Gazebo demonstration at speed cap 6

Terminal 1:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch xycar_rl rl_sim.launch.py \
  headless:=gui \
  enable_rviz:=false
```

Terminal 2:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

MODEL="$PWD/src/xycar_rl/models/lap_time_speed_only_round02_20260723/camera_speed_lap_time_speed_only_actor.pth"

ros2 run xycar_rl rollout_policy \
  --project-root "$PWD" \
  --policy-kind camera_speed_td3_bc \
  --checkpoint "$MODEL" \
  --episodes 1000 \
  --max-steps 600 \
  --control-rate-hz 7 \
  --min-speed-command 4 \
  --max-speed-command 100 \
  --speed-cap-command 6 \
  --start-progress-fraction 0 \
  --recovery-probability 0 \
  --s-curve-focus-probability 0 \
  --action-noise 0 \
  --speed-action-noise 0 \
  --speed-action-bias 0 \
  --disable-adaptive-steering \
  --steering-temporal-alpha 1.0 \
  --speed-temporal-alpha 1.0 \
  --device cuda
```

Each completed, failed, or time-limited episode resets the vehicle and starts
the next lap automatically.
