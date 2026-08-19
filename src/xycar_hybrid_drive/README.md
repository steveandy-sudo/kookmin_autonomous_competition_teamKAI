# Xycar Hybrid Drive

This package publishes the final `/xycar_motor` command from one ROS node.
There is no rule-command topic, policy-command topic, and downstream arbiter
chain in the actuation path.

## Modes

Priority is fixed:

1. `CONE_RULE`: LiDAR cone corridor, adapted from `teamkai/hwj`
2. `LANE_RULE_CURVE`: continuous canonical-path rule at command 25
3. `MODEL_STRAIGHT`: recovery-trained temporal camera-speed policy on straight
   sections at up to 25

The canonical preview enters and exits curve mode after one confirmed frame;
the minimum curve hold and aligned-exit checks prevent mode chatter. Cone mode requires
three consecutive valid LiDAR corridor paths and remains active through short
dropouts. It exits only after the cone path is stale and canonical lane recovery
has been observed repeatedly.

For curve frames, the node computes the canonical rule path first and skips
neural-policy inference entirely. This removes model inference time from the
curve command path. A straight recovery frame restarts the temporal policy
before the mode returns to `MODEL_STRAIGHT`.

The model command is passed through a straight-only deadband, low-pass, and
rate limiter. Curves retain the rule controller's continuous steering values;
the optional three-level pulse controller is disabled. Gazebo uses its CAD
track pose only for route classification and a measured-steering-map
Pure-Pursuit rule with a `1.20 m` lookahead. That simulation-only feedback is
disabled in the real configuration, where the camera-derived canonical rule
remains the curve controller. The real path target is `0.0 m`, exactly 10 cm left of the
previous `+0.10 m` right-offset calibration.

The default straight checkpoint is
`straight_speed25_recovery_v3_20260805/camera_speed_td3_bc_best.pth`. In its
normal straight evaluation it completed all 27 executed episodes at mean speed
`24.985`, with no large oscillation event. The final speed-25 hybrid setting
then completed three consecutive Gazebo laps; the two instrumented repeats
took `12.798 s` and `12.796 s` from first motion to 98% lap progress, with
maximum CAD-relative cross-track errors of `0.203 m` and `0.285 m`.

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
  model_speed_cap:=25.0 \
  cone_speed_cap:=9.5
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
  model_speed_cap:=25.0 \
  cone_speed_cap:=9.5
```

The launch defaults to `drive_enabled:=false`. Command 25 and the continuous
curve outputs must remain in shadow/lifted-wheel testing until the steering
sign, emergency stop, and every left-right mode transition have been checked.

## Simulation

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch xycar_hybrid_drive hybrid_sim.launch.py \
  headless:=gui \
  enable_rviz:=true \
  model_speed_cap:=25.0 \
  cone_speed_cap:=9.5
```

The competition world may not contain a cone corridor. Lane/model mode
switching can still be checked there; cone mode requires cone-sized LiDAR
clusters with a `0.68-0.98 m` corridor. The CAD Pure-Pursuit feedback mentioned
above is a Gazebo validation aid and is never enabled by the real launch.

## Debug Topics

- `/hybrid/mode`: selected mode and reason
- `/hybrid/debug`: selected, model, rule, cone candidates and timing
- `/hybrid/cone_clusters`: accepted LiDAR cone clusters
- `/hybrid/cone_path`: cone corridor center path
- `/xycar_motor_shadow`: the single final command in shadow mode
