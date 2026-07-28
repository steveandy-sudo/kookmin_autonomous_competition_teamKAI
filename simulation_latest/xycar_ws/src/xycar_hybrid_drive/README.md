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
cd /home/xytron/kookmin_ty/simulation_latest
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  xycar_rule_drive xycar_rl xycar_hybrid_drive xycar_final_drive \
  --symlink-install
source install/setup.bash
```

Start the MJPEG camera and `/scan` LiDAR before the hybrid launch. The
low-latency canonical perception is started by `final_real_stack.launch.py`.

Start in shadow mode first:

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=hybrid \
  drive_enabled:=false \
  target_lateral_offset_m:=0.10 \
  model_speed_cap:=10.0 \
  cone_speed_cap:=10.0
```

Confirm that only one motor publisher will exist before enabling:

```bash
ros2 topic info /xycar_motor -v
ros2 topic echo /hybrid/mode
ros2 topic echo /xycar_motor_shadow
```

After the lifted-wheel, steering-sign, mode-transition, and emergency-stop
checks pass:

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=hybrid \
  drive_enabled:=true \
  target_lateral_offset_m:=0.10 \
  model_speed_cap:=10.0 \
  cone_speed_cap:=10.0
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
  model_speed_cap:=10.0 \
  cone_speed_cap:=10.0
```

The current competition world may not contain a cone corridor. Lane/model mode
switching can still be checked there; cone mode requires cone-sized LiDAR
clusters with a `0.68-0.98 m` corridor.

## Debug Topics

- `/hybrid/mode`: selected mode and reason
- `/hybrid/debug`: selected, model, rule, cone candidates and timing
- `/hybrid/cone_clusters`: accepted LiDAR cone clusters
- `/hybrid/cone_path`: cone corridor center path
- `/xycar_motor_shadow`: the single final command in shadow mode
