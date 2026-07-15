# Packaged drive policies

## Canonical BEV policy (current sim-to-real candidate)

`drive_canonical_policy_scripted.pt` is the model used by
`real_canonical_policy_drive.launch.py` and the default simulation policy launch.

- training samples: 30,000 canonical simulation frames from 6 sessions
- general drive: 21,547
- recovery: 8,453
- injected real-camera canonical artifacts: 1,041 events
- stopped rows: 0
- model: ResNet18 image encoder plus LiDAR 1D CNN
- image input: canonical RGB `1x3x90x160`
- LiDAR input: distance and validity `1x2x360`
- output: normalized steering only
- validation MAE: 3.834 Xycar angle command
- held-out test MAE: 3.379 Xycar angle command
- held-out recovery MAE: 4.284 Xycar angle command
- SHA-256: `dd8cb6c2ccfca5a438b08e90f930f50527a88b5169cae9e8239dd129f78dbb21`

The matching report is `drive_canonical_policy_metrics.json`. The model expects
`/perception/canonical_road_image`, not raw camera RGB.

## Legacy raw RGB policy

`drive_policy_scripted.pt` is retained for reproducing the older raw-camera
experiments. It is not the default sim-to-real policy.

- training samples: 73,553 from 14 simulation sessions
- validation MAE: 2.39 Xycar angle command
- held-out test MAE: 2.18 Xycar angle command
- SHA-256: `960d5dcb64d7cae42f23e1038d36c7d866c510f35bab2f5240f898283ff1fbfa`

Speed, steering limits, command smoothing, synchronization and sensor timeout
remain outside both models. Read the repository root `README.md` before enabling
motor output on a real vehicle.
