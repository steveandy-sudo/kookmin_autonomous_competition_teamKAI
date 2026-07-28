# Real Vehicle Rule vs Learned Policy Analysis

Date: 2026-07-24

## Bags used

- Rule:
  `rule_speed8_run01_20260724_131500/bag/bag_0.db3`
- Learned policy:
  `lap_time_4to100_run01_20260724_133049/bag/bag_0.db3`
- `rule_speed8_run02_20260724_131815` was not used for control analysis.
  It contains only 3.36 seconds of sensor data and no motor command or rule
  diagnostic messages.

The rule and model runs used byte-identical canonical adapter parameters. The
recorded command was speed 8 in both runs, and VESC telemetry confirms almost
the same physical speed.

## Main measurements

| Measurement | Rule | Learned policy |
| --- | ---: | ---: |
| Active control duration | 16.58 s | 4.92 s |
| Command rate | 6.93 Hz | 4.47 Hz |
| Mean command interval | 144 ms | 224 ms |
| Command intervals over 200 ms | 0.9% | 50.0% |
| Mean actual VESC speed | 0.639 m/s | 0.641 m/s |
| Mean raw camera age | 30 ms | 31 ms |
| Mean canonical arrival age | 157 ms | 164 ms |
| Mean image age at command | 175 ms | 249 ms |
| Estimated command-to-yaw delay | 200 ms | 160 ms |
| Estimated camera-to-yaw delay | 371 ms | 414 ms |
| Distance traveled during that delay | 0.237 m | 0.265 m |
| Steering at or above 35 command | 18.1% | 52.2% |
| Yellow missing while control active | 0.9% | 33.3% |
| White missing while control active | 0.0% | 0.0% |

The command-to-yaw values are correlation estimates from command and IMU yaw
rate, not measured steering feedback.

## Rule-based run

Perception is not the primary failure in this run. During active control, the
white boundary is present in every canonical frame and yellow is present in
99.1% of frames. The rule node also reports a valid path for every command.

The important problems are:

1. The path span averages only 0.58 m and falls to 0.10 m, even though the
   canonical forward range and configured lookahead are 1.5 m. In tight curves
   the controller therefore often has only a short local path.
2. The image used for a command is already 175 ms old on average.
3. The measured command-to-yaw response adds about 200 ms.
4. The configured `control_latency_preview_sec` is only 0.10 s,
   `steering_lead_time_sec` is 0, and temporal ego compensation is disabled.
5. The result is a late correction that reaches about -41 command after the
   vehicle has already advanced roughly 24 cm.

This explains why the same controller looks acceptable on a straight but is
late and aggressive on a curve.

## Learned-policy run

The learned policy has the same perception pipeline but an additional runtime
timing failure:

1. Canonical input arrives at 6.83 Hz, but policy output is only 4.47 Hz.
2. Half of all command gaps exceed 200 ms and the maximum gap is 575 ms.
3. The temporal model receives frames with a median inferred separation of
   about 231 ms and a maximum of 637 ms. Its simulation training contract was
   a regular 7 Hz sequence, approximately 143 ms apart.
4. Inference itself averages 45 ms. Most of the 249 ms image age is therefore
   upstream perception, queueing, rate-limit interaction, and scheduling.
5. `max_inference_rate_hz=7.0` is too close to the approximately 7 Hz input.
   Arrival jitter can make the callback rate limiter reject the next frame and
   wait for another one.
6. Adaptive steering, preview steering, and temporal output smoothing were all
   disabled. The raw policy command spends 52.2% of the run above magnitude 35.
7. The command sweeps from about -40 to +27 and back to -38. Yellow then leaves
   the canonical view, while the white boundary remains present.

The initial curve failure is therefore mainly a stale and irregular temporal
input plus an unsmoothed, saturated steering response. Yellow loss becomes
worse after the vehicle has already departed from the intended lane.

## Recommended order of fixes

1. Restore a stable 7 Hz policy output:
   set the inference ceiling above the source rate, use a keep-last-1 image
   subscription, and always drop old frames.
2. Record and enforce image freshness. For initial real tests, reject or slow
   down when image age is over 150 ms.
3. Reduce canonical pipeline latency. The raw camera is about 30 ms old, while
   canonical arrival is about 160 ms old.
4. Match temporal training and deployment intervals. Retrain with measured
   real timing jitter or resample to a fixed 7 Hz before creating two-frame
   model input.
5. For the rule controller, compensate the measured total delay and lengthen
   the usable path through curves before changing steering gains.
6. Re-test at speed commands 4, 6, and 8. Increase speed only after curve
   completion succeeds at each level.

## Reproduce the analysis

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/analyze_real_rule_vs_model_bags.py \
  --rule-bag "$PWD/analysis/real_vehicle_rule_vs_model_20260724/김태윤/rule_speed8_run01_20260724_131500/bag/bag_0.db3" \
  --model-bag "$PWD/analysis/real_vehicle_rule_vs_model_20260724/김태윤/lap_time_4to100_run01_20260724_133049/bag/bag_0.db3" \
  --output-dir "$PWD/analysis/real_vehicle_rule_vs_model_20260724/results"
```

Generated artifacts include `summary.json`, command CSV files, camera/canonical
montages, and synchronized time-series plots.

## Remaining measurement gap

The bags contain commanded steering but no measured front-wheel steering
angle, and they contain no odometry suitable for track-relative cross-track
error. IMU yaw rate and VESC speed are available. A future recording should add
measured steering position if possible and a track-relative pose or external
video marker so curve-entry error can be measured directly.
