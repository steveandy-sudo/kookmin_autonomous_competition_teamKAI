# YOLO26n 256 Lane Segmentation Experiment

## What was trained

The Roboflow archive downloaded as `*.yolo26.zip` was used directly after
fixing its relative dataset paths and normalizing the class names.

- model: pretrained `yolo26n-seg.pt`
- classes: `0=white_boundary`, `1=yellow_centerline`
- input: 256x256
- train: 1,463 images
- validation: 411 images
- test: 202 images
- selected run: 72 epochs, stopped automatically with patience 15
- packaged PT: `kookmin_lane_yolo26n_256.pt`
- packaged ONNX: `kookmin_lane_yolo26n_256.onnx`, opset 17, fixed batch 1

YOLO11 and YOLO26 segmentation use the same polygon label syntax. The new
archive contains the same 2,076 source frames as the earlier export, but
Roboflow assigned them to different train, validation, and test splits.
Therefore metrics from the two exports are useful references, not a strict
architecture-only comparison.

## New-export results

Held-out test split from the downloaded YOLO26 export:

| Metric | Result |
|---|---:|
| all mask mAP50 | 0.734 |
| all mask mAP50-95 | 0.462 |
| white mask recall | 0.842 |
| yellow mask recall | 0.494 |

Four-thread production-wrapper benchmark on the development CPU using one
1280x1024 camera frame:

| Format | Mean latency | P95 | Throughput |
|---|---:|---:|---:|
| PyTorch | 13.24ms | 16.06ms | 75.51 FPS |
| ONNX Runtime | 66.59ms | 72.36ms | 15.02 FPS |

ONNX is provided for ASUS-specific testing but is not the default because it
was substantially slower in this environment.

## Vehicle test

Build and source the perception package:

```bash
cd ~/kookmin_ty
source /opt/ros/humble/setup.bash
colcon build --packages-select kaiev26_msgs xycar_perception --symlink-install
source install/setup.bash
```

Run the YOLO26n PT candidate without changing the launch-file default:

```bash
MODEL="$(ros2 pkg prefix xycar_perception)/share/xycar_perception/models/kookmin_lane_yolo26n_256.pt"

ros2 launch xycar_perception real_yolo_canonical_asus.launch.py \
  yolo_model_path:="$MODEL" \
  yolo_image_size:=256 \
  yolo_device:=cpu \
  yolo_cpu_threads:=4
```

Benchmark PT on the ASUS before enabling a driving node:

```bash
ros2 run xycar_perception benchmark_yolo_lane \
  --model "$MODEL" \
  --source /path/to/one/1280x1024_camera_frame.jpg \
  --device cpu \
  --image-size 256 \
  --cpu-threads 4 \
  --no-retina-masks \
  --warmup 10 \
  --iterations 100
```

The normal ASUS launch remains on YOLO11n 256. The YOLO26n candidate did not
show enough yellow-centerline recall to justify replacing the current default
without a live straight, curve, glare, and one-boundary-only vehicle test.

## Checksums

```text
kookmin_lane_yolo26n_256.pt
fb98d00d7abe0224b1d689cd78c71263db0169fddcd3f4e93bcaba122ad289e1

kookmin_lane_yolo26n_256.onnx
54362594f074cb91badb660f56e55808844cb00b28f886ddadb92ae8354c8fce
```
