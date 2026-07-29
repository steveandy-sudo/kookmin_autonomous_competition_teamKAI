# IMU yaw calibration, 2026-07-26

Source session:

```text
/home/xytron/xycar_test_results/imu_yaw/imu_yaw_fixed_20260726_170031
```

The corrected ROS quaternion ordering did not make the sensor-provided
orientation yaw follow physical vehicle yaw. The quaternion result changed by
only -1.15, -0.52, +1.39, and -0.62 degrees during the 90/90/180/360 degree
tests.

The same session's Z-axis angular velocity did follow the physical turns.
Subtracting the stationary median bias of `+0.030 rad/s` produced:

| Test | Expected | Raw gyro Z integral | After sign correction |
|---|---:|---:|---:|
| Left 90 | +90.0 deg | -90.81 deg | +90.81 deg |
| Right 90 | -90.0 deg | +89.87 deg | -89.87 deg |
| Left 180 | +180.0 deg | -182.55 deg | +182.55 deg |
| Left 360 | +360.0 deg | -362.44 deg | +362.44 deg |

The least-squares scale from all four stationary rotation tests is
`0.99220084`. A subsequent same-heading corridor revisit accumulated 748
degrees over two physical 360-degree turns. Production odometry therefore
uses the field-refined scale `0.9922 * 720 / 748 = 0.955`:

```yaml
imu_yaw_source: gyro_z
gyro_yaw_sign: -1.0
gyro_yaw_scale: 0.955
gyro_z_bias_rad_s: 0.030
gyro_deadband_rad_s: 0.015
```

The original quaternion orientation remains published for inspection, but it
is not used as the planar heading source.
