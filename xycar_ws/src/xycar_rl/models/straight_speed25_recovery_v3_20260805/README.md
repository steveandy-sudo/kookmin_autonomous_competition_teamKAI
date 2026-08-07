# Straight speed-25 recovery v3

Selected temporal camera-speed TD3+BC checkpoint for straight sections in the
hybrid driver. Curves and S-curves remain rule-controlled.

- checkpoint: `camera_speed_td3_bc_best.pth`
- selected epoch: 29
- temporal frames: 2
- learned speed range: 4-25
- target right offset: `0.0 m`
- normal straight evaluation: 27/27 completed, mean command 24.985
- large straight oscillation events: 0
- recovery evaluation: 72/100 completed

Only the selected deployment checkpoint is versioned. Intermediate epoch
snapshots and training datasets are intentionally excluded from Git because
they are reproducible artifacts and would add hundreds of megabytes.

Real-vehicle launches remain disarmed by default. Verify the steering sign,
camera calibration, emergency stop, and curve-to-straight transitions in
shadow and lifted-wheel tests before enabling motor output.
