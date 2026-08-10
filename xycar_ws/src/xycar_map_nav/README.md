# Xycar integrated rule drive

This package contains only the real-car integrated drive path:

1. the lane RULE candidate,
2. four-lamp traffic-light control (`red_4`, `yellow_4`, `green_4`,
   `left_4`),
3. the `left_4`-triggered shortcut candidate,
4. cone slalom override,
5. YOLO plus LiDAR vehicle avoidance override,
6. final mode arbitration and the space-bar safety gate.

It intentionally does not install SLAM, localization, Nav2, map, or waypoint
navigation nodes. In rule-only mode the authority order is:
`TRAFFIC_LIGHT(STOP/SHORTCUT) > CONE_RULE > YOLO_LIDAR_AVOIDANCE > RULE`.

The object detector at
`study/my_rule/models/kookmin_objects_best_20260804.pt` preserves the exact
four-lamp class names. Close `red_4` and `yellow_4` boxes latch a stop; two
close `green_4` frames restore normal arbitration. `left_4` must first be
confirmed and then disappear for the configured approach interval before the
selector enables `track_drive_sve.shortcut_candidate_node`. That node reuses
the existing `ShortcutCore` sequence: fixed left entry, lane-guided shortcut
cruise, yellow-T detection, and fixed left exit. The box-area thresholds are
monocular tuning parameters, not metric distances. The SPACE gate and the
integrated `+-42` steering envelope remain authoritative over the candidate.

The shortcut approach delay is measured from the first consecutive detector
frame without `left_4` and advances at the 20 Hz selector rate. A second
consecutive missing frame is still required before entry may start. Set it
without editing source code:

```bash
SHORTCUT_START_DELAY_SEC=0.50 \
  ./src/xycar_map_nav/scripts/run_complete_rule_only.sh

# Or use the fifth positional argument:
./src/xycar_map_nav/scripts/run_complete_space_hybrid.sh 16 0.3 20 12 0.75

# Direct ROS launch:
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  shortcut_start_delay_sec:=0.50
```

Build from the workspace root:

```bash
colcon build --symlink-install --packages-up-to \
  wide_camera xycar_vesc_driver my_rule_msgs my_rule xycar_map_nav
```

Run the real-car rule-only stack from the source tree:

```bash
./src/xycar_map_nav/scripts/run_complete_rule_only.sh
```

After the 2026-08-06 BEV zero-point calibration, the default static lane target
is 9 cm left of the perceived yellow line. This is the conservative starting
value that preserves the previously validated physical trajectory.
Dynamic avoidance offsets are added only while the avoidance state is active.
