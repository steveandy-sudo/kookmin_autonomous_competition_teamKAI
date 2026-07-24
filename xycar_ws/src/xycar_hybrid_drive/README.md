# Xycar Hybrid Drive

This package publishes the final `/xycar_motor` command from one ROS node.
There is no rule-command topic, policy-command topic, and downstream arbiter
chain in the actuation path.

## Modes

Priority is fixed:

1. `CONE_RULE`: LiDAR cone corridor, adapted from `teamkai/hwj`
2. `LANE_RULE_CURVE`: canonical Stanley/Pure-Pursuit on curves
3. `MODEL_STRAIGHT`: temporal camera-speed policy on straight sections

The canonical preview enters curve mode after one frame. Returning to the model
requires three straight frames, which prevents mode chatter. Cone mode requires
three consecutive valid LiDAR corridor paths and remains active through short
dropouts. It exits only after the cone path is stale and canonical lane recovery
has been observed repeatedly.

For curve frames, the node computes the canonical rule path first and skips
neural-policy inference entirely. This removes model inference time from the
curve command path. A straight recovery frame restarts the temporal policy
before the mode returns to `MODEL_STRAIGHT`.

The cone planner retains the `hwj` sequence:

`LaserScan -> DBSCAN clusters -> left/right cone groups -> corridor midpoint
path -> Pure Pursuit -> measured wheel-angle-to-Xycar-command conversion`

The reference implementation was read from
`steveandy-sudo/kookmin_autonomous_competition_teamKAI`, branch `hwj`,
commit `30a4b6a`. Its cone publishers were removed so the calculation runs
inside this package's single final-command node.

## Real Vehicle

Build:

```bash
cd ~/kookmin_ty
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  xycar_rule_drive xycar_rl xycar_hybrid_drive xycar_final_drive \
  --symlink-install
source install/setup.bash
```

Start in shadow mode first:

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=hybrid \
  drive_enabled:=false \
  model_speed_cap:=8.0 \
  cone_speed_cap:=9.5
```

Confirm that only one motor publisher will exist before enabling:

```bash
ros2 topic info /xycar_motor -v
ros2 topic echo /hybrid/mode
ros2 topic echo /xycar_motor_shadow
```

When `xycar_map_nav` owns final actuation, keep this hybrid node in shadow
mode. The map navigator accepts `/xycar_motor_shadow` only while its configured
segment is `cone_rule`; outside that segment it ignores cone false positives.
Never enable both nodes as `/xycar_motor` publishers.

After the lifted-wheel, steering-sign, mode-transition, and emergency-stop
checks pass:

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=hybrid \
  drive_enabled:=true \
  model_speed_cap:=8.0 \
  cone_speed_cap:=9.5
```

The launch defaults to `drive_enabled:=false`. Raise either speed cap only after
recording a successful low-speed bag for all three mode transitions.

## Simulation

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch xycar_hybrid_drive hybrid_sim.launch.py \
  headless:=gui \
  enable_rviz:=true \
  model_speed_cap:=8.0 \
  cone_speed_cap:=9.5
```

The generated SLAM world contains a lower-right cone corridor. It uses the
repository's fixed `1.632 m` road width, a `0.85 m` cone corridor, and the
reference cone size `0.18 x 0.18 x 0.36 m`. The map-frame handover contract is
installed as `config/cone_rule_zone_slam_map.yaml`.

Inside that zone, keep SLAM localization running but ignore the coarse global
path for actuation. `CONE_RULE` owns steering and speed until the cones end and
lane recovery is confirmed. This prevents individual movable cones from being
treated as permanent SLAM-map obstacles.

## Debug Topics

- `/hybrid/mode`: selected mode and reason
- `/hybrid/debug`: selected, model, rule, cone candidates and timing
- `/hybrid/cone_clusters`: accepted LiDAR cone clusters
- `/hybrid/cone_path`: cone corridor center path
- `/xycar_motor_shadow`: the single final command in shadow mode
