# Packaged drive policy

`drive_policy_scripted.pt` is the camera and LiDAR steering policy packaged for
simulation and real-vehicle inference.

- training samples: 73,553 from 14 simulation sessions
- model: ResNet18 image encoder plus LiDAR 1D CNN
- image input: RGB `1x3x90x160`
- LiDAR input: distance and validity `1x2x360`
- output: normalized steering only
- validation MAE: 2.39 Xycar angle command
- held-out test MAE: 2.18 Xycar angle command
- SHA-256: `960d5dcb64d7cae42f23e1038d36c7d866c510f35bab2f5240f898283ff1fbfa`

Speed, command clamping, sensor timeout, and physical emergency stopping remain
outside the model. See `docs/real_bc_vehicle_runbook.md` before enabling motor
output on the real vehicle.
