# track_run_02 canonical lane analysis

## Scope

- Physical track: team-built temporary track
- Bag: `/home/as/Downloads/track_run_02_extract/track_run_02_20260715_143636`
- Duration: 72.54 seconds
- Raw camera: 1,805 frames at 1280x1024
- Recorded canonical output: 1,133 frames
- Current pipeline replay: 1,802 frames

This track must be treated separately from the real competition track and the
Gazebo track. Camera calibration can be shared while the camera mount is
unchanged, but lane-width topology and track-specific appearance must not be
tuned from this bag and silently applied to the competition profile.

## Recorded output versus current pipeline

| Metric | Bag-recorded 1.2m output | Current 1.5m replay |
|---|---:|---:|
| Processing rate | 15.67Hz | 24.85Hz |
| Two coherent white boundaries | 14.92% | 15.76% |
| One coherent white boundary | 74.49% | 73.36% |
| No coherent white boundary | 10.59% | 10.88% |
| Yellow centerline output | 61.08% | 48.83% |
| Two white boundaries plus yellow | 13.06% | 13.71% |

The old yellow percentage is inflated by false-positive yellow clutter. For
example, the old output around 19 seconds contains broad yellow blobs that are
not centerline tape. The lower current percentage is therefore not by itself a
regression.

## Current-pipeline findings

1. A yellow candidate reaches the topology tracker in 77.7% of frames.
2. Yellow is accepted into the final canonical image in 48.8% of frames.
3. In 28.9% of all frames, a yellow candidate is explicitly rejected by the
   white-lane corridor check.
4. The longest final yellow loss is 3.15 seconds, from 38.80 to 41.94 seconds.
5. The longest period without a valid white pair is 10.02 seconds, from 11.17
   to 21.19 seconds. At least one white line is usually retained; the longest
   complete white loss is only 0.45 seconds.
6. Around 47 seconds the camera is looking outside the road. That section is
   an off-track/recovery event, not a pure lane-detector failure.

## Temporary-track final profile

The final replay uses a separate temporary-track launch profile. It keeps the
competition profile unchanged and applies these temporary-track measurements:

- Expected yellow-to-white distance: 0.49m
- Width tolerance: 0.14m
- Minimum median white-component brightness: HSV V 140
- Yellow component-to-curve gate: 0.08m
- Observation-only tracking: no coasting or synthesized hidden lane

The final replay synchronized all 1,802 source frames. White output presence
increased from 89.12% to 95.45%, yellow output presence increased from 48.83%
to 52.55%, and frames containing both increased from 43.45% to 52.55%.

Two tracker defects were also corrected. Topology now compares a short yellow
dash with a white boundary only over rows where both are visible, instead of
extrapolating the dash over the full white curve. Observation-only mode also
replaces stale curves immediately and never uses a previous-frame yellow curve
to classify current white candidates after its search time has expired.

## Reported frame mapping

The thirteenth submitted image was intended as item 9. The corrected mapping
therefore uses the duplicated `idx=0501` source frame for two different
observations and shifts the former items 9 through 12 down by one.

| No. | Frame | Diagnosis and final result |
|---:|---:|---|
| 1 | 0001 | Yellow remains rejected: its BEV distance from the only white boundary is about 0.88m, outside this track's 0.35-0.63m gate. |
| 2 | 0003 | The short center white impostor is removed. The yellow candidate is still just outside the width gate at about 0.685m. |
| 3 | 0144 | The intersecting second white branch is rejected; the stronger physical white curve remains. |
| 4 | 0317 | The dim seam contribution is reduced while the visible white boundary and yellow dash remain. |
| 5 | 0378 | The isolated false white component is removed completely. |
| 6 | 0459 | Yellow clutter is reduced from five components / 757 pixels to two components / 454 pixels. |
| 7 | 0473 | The visible white boundary and two yellow dash pieces are retained; the dark module gap itself is not colored as a lane. |
| 8 | 0501 | The warm color shift is already present in the source camera image. Canonical output normalizes lane colors, but camera auto white balance must be locked separately for raw-image consistency. |
| 9 | 0501 | This is a physical track break. Observation-only input deliberately does not invent road pixels across it; such intervals should be marked invalid for learning. |
| 10 | 0614 | Valid white and yellow observations resume when the physical road returns. |
| 11 | 0924 | Fixed: the yellow center dash and the matching white boundary are both accepted in the same frame. |
| 12 | 0958 | Fixed: the short false white component is rejected and only the long boundary remains. |
| 13 | 1072 | The module break is not filled. Geometry-supported tape pieces on both sides remain visible and the physical break should be excluded by dataset validity rules. |

## Geometry

When one accepted white boundary and yellow centerline overlap, their median
separation in current canonical coordinates is 0.489m. The 10th-to-90th
percentile range is 0.364m to 0.550m. Frames with a stable white pair and
yellow produce a similar median side-to-center distance of 0.481m.

This suggests a temporary-track white-center spacing near 0.96m to 0.98m in
the current BEV coordinates. It is larger than the competition profile's
0.824m white-center spacing and 0.412m half width. A physical tape-center
measurement is needed before treating 0.98m as ground truth, because residual
BEV error on tight curves also contributes to this estimate.

## Failure causes

- Tight curves frequently move one white boundary outside the camera field of
  view. Observation-only canonical output correctly leaves that side absent.
- The tracker uses the competition-track expected half width of 0.412m. Valid
  temporary-track yellow observations near 0.49m sit closer to the topology
  gate and are rejected when BEV error grows through a curve.
- Bright or desaturated yellow tape is sometimes classified as white. The
  washed-out-yellow recovery is much less reliable when only one boundary is
  visible.
- The old 1.2m output had substantial clutter and geometry distortion. The
  current geometry filter removes most broad reflections, but it can also
  reject a real center dash when the temporary-track topology does not match
  the competition profile.

## Recommended next change

The temporary-track perception profile is now separate from the competition
profile. Confirm its 0.98m white-center spacing with one physical tape-center
measurement before using it as ground truth.

For a shared detector, accept a longitudinal yellow candidate from one visible
white boundary when its measured offset matches the selected track profile.
Do not synthesize the hidden boundary in the observation-only model input.
This preserves the current training contract while reducing valid-yellow
rejection on tight curves.

## Artifacts

- `track_run_02_recorded/canonical_lane_report.json`
- `track_run_02_recorded/canonical_even_samples.jpg`
- `track_run_02_recorded/canonical_failure_samples.jpg`
- `track_run_02_current/canonical_lane_report.json`
- `track_run_02_current/current_tracking_samples.jpg`
- `track_run_02_current/canonical_lane_frames.csv`
- `track_run_02_temp_profile_final_reported_before_after.jpg`
- `../datasets/real_bag_canonical/drive/real_track_run_02_temp_profile_final_20260715/`

Regenerate statistics from any bag containing canonical masks with:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/analyze_canonical_bag.py BAG_PATH \
  --output-dir OUTPUT_DIRECTORY
```
