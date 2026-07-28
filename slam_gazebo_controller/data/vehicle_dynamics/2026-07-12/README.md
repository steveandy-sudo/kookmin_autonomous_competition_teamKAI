# 2026-07-12 real Xycar dynamics calibration

Source: `steveandy-sudo/kookmin_autonomous_competition_teamKAI`, branch `hwj`,
path `data/vehicle_dynamics/2026-07-12`.

The full 5,909-row raw CSV remains on the vehicle PC at
`/home/xytron/kookmin_simulation/vehicle_dynamics.csv`. The GitHub
`vehicle_dynamics.csv` contains only its header, so this repository keeps the
grouped summary that was actually used for simulation calibration.

Applied values:

- `speed_gain`: `0.080191 m/s/cmd`
- `steering_delay_sec`: `0.035 s`
- `speed_delay_sec`: `0.094 s`
- steering: direction-aware piecewise-linear command-to-curvature map
- temporary endpoints: `-42 -> 1.366747 1/m`, `+42 -> -1.860716 1/m`

The `+-42` endpoints are not measurements. They extend each direction's
measured `20-to-30` curvature slope to the previously observed servo saturation
near command magnitude `42`. Commands beyond that range clamp to the endpoint.

The report's `wheel_base_m=0.1823` is an aggregate inverse-model estimate, not
a physical measurement. Group estimates vary from `0.1186` to `0.3544 m`, so
the measured physical wheelbase of `0.32 m` remains in the SDF. The bridge uses
the measured turning curvature directly, avoiding that ambiguity.

The source report's IMU yaw sign is opposite to the previously verified Xycar
motor/Ackermann command convention. Curvature magnitudes and left/right
asymmetry are preserved while signs follow the existing simulation and real
VESC command convention: positive `angle_cmd` turns right.

## Headless Gazebo check

The updated final world and bridge were run headlessly at `speed_cmd=5`.

| command | real median | Gazebo result | error |
| --- | ---: | ---: | ---: |
| speed `5` | `0.4001~0.4005 m/s` | `0.400955 m/s` | `<0.3%` |
| angle `-30`, speed `5` radius | `1.0957 m` | `1.071 m` | `2.3%` |
| angle `+30`, speed `5` radius | `0.7429 m` | `0.771 m` | `3.8%` |

Temporary `+-42` extrapolation check:

| command | extrapolated radius | Gazebo result |
| --- | ---: | ---: |
| angle `-42`, speed `5` | `0.732 m` | `0.719 m` |
| angle `+42`, speed `5` | `0.537 m` | `0.597 m` |

The larger positive-side mismatch is consistent with stronger contact slip at
the extrapolated steering extreme. These endpoints should be replaced, not
further curve-fitted, once real `+-40/+-42` runs are available.

This dataset does not contain a reliable acceleration rise curve, braking
curve, measured mass/center of gravity, or tire slip test. The existing
`+-2 m/s^2` acceleration limits, mass, inertia, and contact friction therefore
remain provisional instead of being tuned from unsupported assumptions.
