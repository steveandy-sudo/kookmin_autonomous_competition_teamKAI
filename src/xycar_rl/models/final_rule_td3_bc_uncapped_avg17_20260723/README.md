# Final camera-only TD3+BC

Selected checkpoint: `camera_speed_td3_bc_best.pth`, epoch 14.

```text
input: previous + current 256x144 canonical image
output: steering and speed command
speed normalization range: 4..24
additional deployment cap: disabled (0.0)
SHA256: 6a221ab238097a48342a1abf144326d7c84d51597f5a18455cc1c81598b464d1
```

Gazebo validation:

```text
fixed start:     5/5 lap complete, mean speed command 18.46..18.63
recovery start:  5/5 lap complete, mean speed command 18.51..18.76
recovery CTE:    maximum 0.247..0.265m
observed maximum speed command: 23.16
```

Real-car shadow:

```bash
ros2 launch xycar_rl final_uncapped_avg17_real.launch.py \
  drive_enabled:=false device:=cpu
```

The dedicated launch keeps the trained 7Hz action contract:

```text
adaptive_steering_enabled=false
steering_temporal_alpha=1.0
speed_temporal_alpha=1.0
max_inference_rate_hz=7.0
```

Do not enable real motor output before canonical-image, shadow-command,
steering-sign, wheels-off-ground, emergency-stop, straight, and curve checks.
