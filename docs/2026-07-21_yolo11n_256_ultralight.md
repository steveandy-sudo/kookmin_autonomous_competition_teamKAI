# YOLO11n 256 Ultralight Lane Model

## Goal

Reduce lane-segmentation CPU load on the Ryzen 5 vehicle computer while
keeping the existing YOLO mask, BEV, and canonical pipeline unchanged.

## Dataset and training

- classes: `white_boundary`, `yellow_centerline`
- train: 1,543 images
- validation: 397 images
- test: 136 images
- initialization: previous YOLO11n-seg 512 best checkpoint
- training input: 256x256
- selected checkpoint: validation mask mAP50-95 0.496
- packaged inference checkpoint: 6.0 MB

## Independent test comparison

| Metric | YOLO11n 512 | YOLO11n 256 |
|---|---:|---:|
| all mask mAP50 | 0.880 | 0.806 |
| all mask mAP50-95 | 0.566 | 0.516 |
| white mask recall | 0.969 | 0.954 |
| yellow mask recall | 0.718 | 0.625 |

The 256 model trades distant yellow-dash recall for lower latency. The
canonical tracker may bridge short gaps, but it must not be treated as a
replacement for a visible observation during final vehicle validation.

## Four-thread CPU wrapper benchmark

The same 1280x1024 real-camera frame and production wrapper were used.

| Profile | Mean | Median | P95 | Throughput |
|---|---:|---:|---:|---:|
| 512, retina masks | 24.51ms | 23.11ms | 35.20ms | 40.80 FPS |
| 256, light masks | 13.30ms | 13.02ms | 15.89ms | 75.17 FPS |

The development PC is faster than the vehicle PC. Use the ratio for comparison
and run `scripts/check_asus_yolo_runtime.sh` on the ASUS before driving.

## Vehicle launch

The normal ASUS launch selects the 256 model, four CPU threads, at most eight
instances, and low-cost mask resizing:

```bash
ros2 launch xycar_perception real_yolo_canonical_asus.launch.py
```

Accuracy fallback using the packaged 512 checkpoint:

```bash
MODEL_512="$(ros2 pkg prefix xycar_perception)/share/xycar_perception/models/kookmin_lane_yolo11n_512.pt"

ros2 launch xycar_perception real_yolo_canonical_asus.launch.py \
  yolo_model_path:="$MODEL_512" \
  yolo_image_size:=512 \
  yolo_max_detections:=30 \
  yolo_retina_masks:=true
```

Run perception without a motor or policy node first. Verify the raw rectified
view, YOLO mask, and canonical image over straight, curved, reflective, and
one-boundary-only sections before enabling vehicle commands.
