# Jetson final competition stack

This package owns the single ROS 2 launch entry point used on the Jetson Orin
NX competition computer. Runtime source, launch files, configuration, and
packaged models remain under `xycar_ws/src`. Generated `build`, `install`, and
`log` trees and the Python virtual environment are not source artifacts and
must not be committed.

## Jetson build environment

The current JetPack 6.2.1 setup uses these external, machine-local
dependencies:

- Python virtual environment: `~/kookmin_ty/venvs/xycar-jp621`
- YDLidar SDK 1.2.7 prefix:
  `~/kookmin_ty/ydlidar-sdk-v1.2.7-install`

Build console scripts with the virtual-environment Python so perception and
policy nodes keep access to the Jetson CUDA build of PyTorch:

```bash
cd ~/kookmin_ty/jetson_final/xycar_ws
source /opt/ros/humble/setup.bash
source ~/kookmin_ty/venvs/xycar-jp621/bin/activate
export CMAKE_PREFIX_PATH="$HOME/kookmin_ty/ydlidar-sdk-v1.2.7-install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
python -m colcon build --packages-up-to xycar_final_drive --symlink-install
source install/setup.bash
```

Do not replace the virtual environment with generic desktop CUDA/PyTorch
packages. Do not commit the virtual environment or generated TensorRT engine
files.

## Competition entry point

The complete real-car stack is started with one command:

```bash
ros2 launch xycar_final_drive final.launch.py
```

The launch starts:

1. the wide MJPEG camera;
2. the native ROS 2 YDLidar driver;
3. the native ROS 2 VESC driver;
4. CUDA LR-ASPP lane perception; and
5. the validated hybrid rule/RL driver.

Both motor safety gates default to false:

- `command_drive_enabled:=false` prevents the selected driver from publishing
  `/xycar_motor`;
- `vesc_drive_enabled:=false` prevents the VESC driver from applying non-zero
  output.

Keep both false for the first hardware start. The default Torch device is
`cuda`, the default learned-policy cap is command 20, and the cone cap is 9.5.

## Hardware prerequisites

Before starting the complete launch, identify and validate each device
individually:

- camera device matching `camera_device`;
- `/dev/ttyLIDAR` using the packaged `ydlidar.yaml` profile;
- `/dev/ttyMOTOR` at 115200 baud;
- VESC firmware 2.18, fresh telemetry, safe battery voltage, and no faults.

The repository does not yet install machine-specific udev rules. Create them
only after recording each physical device's VID, PID, and serial number. The
runtime user must also have serial-port access, normally through `dialout`.

## Shadow validation

With the hardware connected but the wheels unable to drive the car:

```bash
ros2 topic info /xycar_motor -v
ros2 topic echo /xycar_motor_shadow
ros2 topic hz /perception/canonical_road_image
ros2 topic hz /scan
ros2 topic hz /vehicle/vesc_state
ros2 topic echo /diagnostics
```

Enable the command publisher while leaving VESC output disabled only after
camera and LiDAR orientation, steering sign, timestamps, and failure behavior
have been checked:

```bash
ros2 launch xycar_final_drive final.launch.py \
  command_drive_enabled:=true vesc_drive_enabled:=false
```

Actual VESC output requires both gates to be true. Do not enable both until the
lifted-wheel test, emergency stop, single-publisher check, and sensor-loss stop
behavior have passed on the connected car.
