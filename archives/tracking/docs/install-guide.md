# Tracking installation

`tracking/` is the monorepo's ROS 2 workspace. Calibration, TCP receiver,
C++ bridge, IMU/camera drivers, Odin integration and recording source are owned
here. Python uses only the root `.venv`; do not create a tracking environment.

## System and ROS SDK prerequisites

Install a supported ROS 2 Humble SDK separately (normally `/opt/ros/humble`).
Set `ROS_SETUP=/absolute/path/to/setup.bash` for another SDK installation.
Create the root `.venv` with that SDK's Python interpreter and
`--system-site-packages`, then follow the root installation guide. Ubuntu 22.04
apt Humble uses Python 3.10; Python 3.11 cannot load its `rclpy` binary extension.
The root environment must include `colcon-common-extensions`, `setuptools<80`,
NumPy, PyYAML, h5py and MuJoCo. Direct non-ROS viewers also need SciPy,
Matplotlib and OpenCV; the Rerun viewer needs `rerun-sdk`.

Alternatively, the root Pixi `tracking` feature provisions a compatible
RoboStack ROS SDK. Set `ROS_SETUP` to the root
`.pixi/envs/tracking/setup.bash` and create `.venv` with its Python as described
in the root guide. Pixi supplies native dependencies only; collection and
calibration Python still run from the single root `.venv`.

System dependencies: C++ compiler, CMake (<4), make or Ninja, pkg-config,
Eigen3, OpenCV, HDF5, PCL, yaml-cpp, OpenSSL, libusb, `adb`, `tmux`, and
util-linux (`flock`, `setsid`). ROS dependencies are declared in each
`src/*/package.xml`: rclcpp/rclpy, ament/colcon/rosidl, launch/launch_ros,
standard geometry/sensor/navigation/visualization/stereo messages and services,
cv_bridge, image_transport, tf2/tf2_ros/tf2_geometry_msgs, message_filters,
pcl_conversions and rcpputils. Tests additionally need pytest and ament gtest.

The headset must run the compatible `pico_wholebody_stream.apk` streaming app
and authorize USB debugging. That hardware/vendor application is not built by
this repository. Obtain it from the device supplier if unavailable. IMU900,
V4L2 cameras and Odin sensors are optional hardware inputs, not emulated SDKs.

## Build from the monorepo root

```bash
bash tracking/scripts/build.sh          # PICO, IMU900, cameras, collector
bash tracking/scripts/build.sh --all    # also Odin1, Odin Lite and pelvis fusion
source tracking/scripts/environment.sh
```

The build writes only `tracking/build`, `tracking/install` and `tracking/log`.
Do not reuse generated overlays or environments from an older checkout.
The C++ Odin Lite driver links `tracking/src/odin/odin-sdk2` directly. The
build compiles this supplied SDK locally; no git submodules, symlink checkout,
or download is involved. Odin1 retains its supplied architecture-specific
vendor static libraries and license. Unsupported vendor binary architectures
require the supplier's matching SDK, not a fabricated implementation.

## Calibration and teleoperation

All commands below run from the monorepo root; launchers select `.venv` and
source the ROS SDK and local tracking overlay themselves.

```bash
bash tracking/scripts/start_pico_driver.sh
# A second terminal: select a side and TCP -> wrist -> arm geometry.
bash tracking/scripts/calibrate_pico_arm.sh --user OPERATOR
bash tracking/scripts/calibrate_pico_arm.sh status --user OPERATOR
# Or individual non-profile calibration stages:
bash tracking/scripts/calibrate_pico_arm.sh left tcp
bash tracking/scripts/calibrate_pico_arm.sh left wrist
bash tracking/scripts/calibrate_pico_arm.sh left geometry
bash tracking/scripts/calibrate_pico_palm_orientation.sh left
```

TCP calibration preserves the four position marks followed by the fifth
orientation mark. Wrist pivot and arm geometry retain their interactive
capture, acceptance gates and explicit artifact dependencies. Geometry also
accepts `--start-mode space|auto`, `--domain`, `--countdown-s` and
`--duration-scale`. Published profile calibration is managed by the root
`teleop_profile.py`; never edit published artifacts in place. Standalone palm
orientation recalibration targets the legacy global calibration only.

```bash
bash tracking/scripts/start_pico_m0.sh --viewer --calibration-dir /absolute/calibration
bash tracking/scripts/start_tianji_pico_teleop.sh --detach --calibration-dir /absolute/calibration
bash tracking/scripts/start_tianji_pico_teleop.sh --status
bash tracking/scripts/stop_tianji_pico_teleop.sh
```

Named calibration directories require both valid arm chains and fail closed;
they never fall back to global artifacts. Without a directory the legacy
`~/.config/pico_tracker` paths remain supported. No launch scans recordings
for a candidate. The tmux pipeline sends TJVR v4 UDP to `127.0.0.1:15000`;
it does not launch the robot controller or send motor commands. Use the root
collection launchers for the full control/retargeting workflow.

For low-level ROS commands first `source tracking/scripts/environment.sh`.
Defaults remain domain 120, localhost-only communication and disabled CLI
daemon. Direct receiver tools in `PICO_adb/` and `reference/` use root Python
and `adb forward tcp:9999 tcp:9999` without ROS. Only one TCP client may own
the headset stream at a time.

## Optional collection and verification

```bash
bash tracking/scripts/start_pico_odin_collect.sh --odin-lite
bash tracking/scripts/record_pico_tremor.sh --dry-run
source tracking/scripts/environment.sh
cd tracking
colcon test --base-paths src --packages-select pico_bridge imu_ros2 data_collector --event-handlers console_direct+
colcon test-result --verbose
```

For a hardware-free receiver smoke test, start
`python tracking/src/pico_bridge/scripts/mock_pico_server.py --fps 5 --duration 30`
from the root in one terminal, then start the built `pico_bridge_node` in
another sourced terminal. Observe `/pico/pose/head` with
`ros2 topic echo --once /pico/pose/head`; stop the owned processes afterward.
A real calibration/teleop acceptance check needs the headset and operator.

Original ignored environments, recordings and logs left in an old checkout
are user data, not runtime dependencies. New captures use `tracking/recordings`
or profile-specific recording directories. The optional ROS collector retains
its existing HDF5/video and `s` start / `d` save / `q` quit behavior.
