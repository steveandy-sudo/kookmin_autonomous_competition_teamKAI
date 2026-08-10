# Xycar integrated rule drive

This package contains only the real-car integrated drive path:

1. the lane RULE candidate,
2. cone slalom override,
3. YOLO plus LiDAR vehicle avoidance override,
4. final mode arbitration and the space-bar safety gate.

It intentionally does not install SLAM, localization, Nav2, map, or waypoint
navigation nodes. In rule-only mode the authority order is:
`CONE_RULE > YOLO_LIDAR_AVOIDANCE > RULE`.

Build from the workspace root:

```bash
colcon build --symlink-install --packages-up-to \
  xycar_camera xycar_vesc_driver my_rule_msgs my_rule my_drive
```

Run the real-car rule-only stack from the source tree:

```bash
./src/my_drive/scripts/run_complete_rule_only.sh
```

After the 2026-08-06 BEV zero-point calibration, the default static lane target
is 9 cm left of the perceived yellow line. This is the conservative starting
value that preserves the previously validated physical trajectory.
Dynamic avoidance offsets are added only while the avoidance state is active.
