# Workspace layout

The monorepo root owns `.venv`, dependency installation and collection entry
points. `tracking/` is an internal ROS workspace, alongside `control/` and
`retargeting/`; none is a separately checked-out repository.

```text
tracking/
  scripts/
    environment.sh                 Root .venv + ROS SDK + tracking overlay
    build.sh                       PICO core / optional complete ROS build
    calibrate_pico_arm.sh           TCP -> wrist pivot -> arm geometry
    start_pico_driver.sh            Headset TCP forwarding and ROS bridge
    start_pico_m0.sh                Strict artifact validation and skeleton
    start_tianji_pico_teleop.sh     Managed driver/M0/TJVR tmux session
  src/
    pico_bridge/                   C++ TCP bridge, fusion, TJVR; calibration UI
    imu_ros2/                      C++ IM900/IM948 serial drivers
    fisheye_camera/                C++ V4L2 camera capture
    data_collector/                Optional ROS HDF5/video collection
    odin_ros_driver/               Odin1 vendor driver and supplied libraries
    odin/
      odin-sdk2/                   Supplied Odin Lite SDK source
      odin_ros_driver2/            C++ driver linking sibling SDK directly
      scripts/                    Local SDK build helpers
    pico_odin/                     C++ pelvis alignment/calibration/fusion
  PICO_adb/                        Standalone TCP receiver and viewers
  reference/                      Original receiver/visualizer reference
  recordings/                     New local captures (ignored)
  build/, install/, log/           Generated ROS artifacts (ignored)
```

From the monorepo root:

```bash
bash tracking/scripts/build.sh
source tracking/scripts/environment.sh
bash tracking/scripts/start_pico_driver.sh
```

Use `build.sh --all` for Odin support. For system SDK dependencies, standalone
receivers, calibration controls, optional sensor inputs and hardware-free
verification, see [install-guide.md](install-guide.md). Existing recordings and
logs in pre-refactor directories remain user data and are not moved into the
runtime source tree.
