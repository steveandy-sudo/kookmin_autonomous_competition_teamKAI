# Real low-latency pipeline measurement

The ASUS Xycar PC was measured on 2026-07-24 with motor output disabled.
The camera published original 1280x1024 MJPEG at about 29.9 Hz. Perception
decoded and rectified only the latest frame at 7 Hz, generated canonical in
the same process, and fed Dual DAgger v6 epoch 51 in shadow mode.

| Perception threads | Canonical Hz | Policy Hz | Perception p95 | Image age at command p95 |
|---:|---:|---:|---:|---:|
| 2 | 6.98 | 6.98 | 99.1 ms | 167.8 ms |
| 4 | 7.00 | 7.00 | 77.3 ms | 133.8 ms |
| 6 | 7.01 | 7.01 | 97.4 ms | 159.6 ms |

Four perception threads are the selected default. In the 30-second run:

- camera: approximately 29.9 Hz
- canonical and policy command: 7.00 Hz
- JPEG decode plus rectification p95: 36.5 ms
- LR-ASPP model p95: 34.7 ms
- BEV plus canonical p95: 22.9 ms
- policy inference p95: 13.2 ms
- stale perception and policy frames: 0

The previous recorded policy output was about 2.16 Hz. The new source-driven
path removes the external 30 Hz rectifier, raw Image transport, canonical
adapter, intermediate synchronization, and the policy's second rate limiter.
