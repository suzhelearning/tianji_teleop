# pico_project

[English](README.md) | [简体中文](README.zh-CN.md)

PICO 4U data collection and foot-orientation fusion pipeline. The ROS 2 Humble
workspace uses the monorepo root `.venv` and includes PICO streaming, dual IMU900 input,
fisheye cameras, Odin odometry, data recording, and MuJoCo visualization.

## Quick start: PICO palm-constrained skeleton

The helper scripts default to `ROS_DOMAIN_ID=120`, `ROS_LOCALHOST_ONLY=1`, and
disable the ROS 2 CLI daemon. Calibration and PICO-only validation run from the
root `.venv` with the installed ROS SDK. Launchers source this environment automatically.

For local PICO and exoskeleton testing, keep the following values identical in
every terminal after entering `source scripts/environment.sh`:

```bash
export ROS_DOMAIN_ID=120
export EXO_REQUESTED_ROS_DOMAIN_ID=120
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
```

Terminal 1 starts the PICO driver:

```bash
cd /path/to/tianji_teleop/tracking
source scripts/environment.sh
./scripts/start_pico_driver.sh
```

For a new glove/operator, terminal 2 opens the unified calibration menu. Choose
one side and either an individual item or the ordered
`TCP -> wrist pivot -> arm geometry` workflow:

```bash
cd /path/to/tianji_teleop/tracking
source scripts/environment.sh
./scripts/calibrate_pico_arm.sh
```

Existing calibration can be inspected without changing it:

```bash
./scripts/calibrate_pico_arm.sh status
```

After calibration, choose the command for the intended workflow. For PICO-only
validation of the palm-constrained skeleton, run:

```bash
./scripts/start_pico_m0.sh --viewer
```

For Tianji PICO teleoperation, first confirm that both sides have completed
`TCP -> wrist pivot -> arm geometry`, then run:

```bash
./scripts/start_tianji_pico_teleop.sh
```

This one-command launcher removes historical teleoperation instances, then
starts the PICO driver, corrected M0 skeleton and its MuJoCo viewer, and the
ROS 2 to TJVR bridge. The driver and M0 commands above do not need to be run
separately. Stop teleoperation with:

```bash
./scripts/stop_tianji_pico_teleop.sh
```

Both runtime paths validate and load saved artifacts for each valid side; they
do not scan `recordings/` for candidates.

The startup is ready only after it prints both `PICO M0 streams ready` and
`PICO bilateral M0 ready`. If one side falls back to raw SMPL, calibrate that
side independently, for example `./scripts/calibrate_pico_arm.sh right all`.

The primary viewer skeleton is `/pico/smpl_palm_corrected_ik`. It has exactly
the same joint positions as `/pico/smpl_palm_corrected`, with only the shoulder
and elbow frame bases adapted for IK. Check the live chain with:

```bash
ros2 topic hz /pico/smpl_palm_corrected
ros2 topic hz /pico/smpl_palm_corrected_ik
ros2 topic echo /pico/smpl_palm_corrected/status --once
```

The exoskeleton consumes only these PICO-side outputs:

```text
/pico/smpl_palm_corrected_ik
/pico/smpl_palm_corrected/status
/pico/tracking_epoch
```

### Start the PICO side of Tianji Spark teleoperation

After both arms have valid calibration artifacts, one command starts the PICO
driver, corrected M0 upper-limb skeleton, its PICO skeleton MuJoCo viewer, and
the ROS 2 to TJVR bridge:

```bash
cd /path/to/tianji_teleop/tracking
./scripts/start_tianji_pico_teleop.sh
```

The launcher manages one tmux session named `pico_tianji_teleop` with three
windows: `driver`, `m0`, and `bridge`. Use `Ctrl-b n` to move to the next
window, `Ctrl-b 0/1/2` to select one, and `Ctrl-b d` to detach without stopping
the pipeline. Every normal start first removes historical driver, M0, and
Tianji bridge processes across old work directories and ROS domains, then
creates one fresh session. Management commands are:

```bash
./scripts/start_tianji_pico_teleop.sh --detach
./scripts/start_tianji_pico_teleop.sh --status
./scripts/stop_tianji_pico_teleop.sh
```

The bridge accepts only timestamp-matched corrected skeleton and status frames.
Its default `robot_arm_segments` mode reconstructs the target with Tianji arm
segment lengths at reach scale `0.95`. Each accepted source frame becomes one
atomic 656-byte bilateral TJVR v4 UDP packet containing palm targets, complete
corrected upper-limb positions and rotations, and left/right arm redundancy
directions with validity flags. It does not resample or apply a clutch.

Inspect input and bridge diagnostics in any tracking ROS terminal:

```bash
ros2 topic hz /pico/smpl_palm_corrected_ik
ros2 topic echo /pico/smpl_palm_corrected/status --once
ros2 topic echo /pico/tianji_mujoco_teleop/status --once
```

The launcher sends to `127.0.0.1:15000` and is intended for the Tianji MuJoCo
evaluation path; it does not command robot hardware.

The M0 window opens the PICO skeleton MuJoCo viewer by default, showing
`/pico/smpl_palm_corrected_ik` with the raw SMPL overlay. The launcher still
does not start the receiving DLS/Spark robot MuJoCo Viewer. Start that viewer
independently when robot-side visual validation is required:

```bash
cd ../control
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_wuji2.xml \
  --algorithm spark_upper_qpoases \
  --pico-teleop \
  --pico-bind 127.0.0.1 \
  --pico-port 15000
```

`--algorithm spark_upper_qpoases` selects Spark-style skeleton scaling and the
two-stage qpOASES position IK. `--pico-teleop` enables live UDP input. If the
overlay still shows `pico_enabled=0`, press `P` once.

For ROS communication with another machine, export `ROS_LOCALHOST_ONLY=0` in
every terminal before running the scripts. Keep `ROS_DOMAIN_ID=120` identical
on all machines.

## Packages (under `src/`)

| Package | Language | Role |
|---------|----------|------|
| `pico_bridge` | C++ (rclcpp) | PICO TCP bridge, IMU900 foot fusion, and MuJoCo viewer. |
| `imu_ros2` | C++ (rclcpp) | IM900/IM948 serial driver with dual-foot configuration. |
| `fisheye_camera` | C++ (rclcpp) | V4L2 USB fisheye cameras to ROS topics. |
| `data_collector` | Python (rclpy) | Saves subscribed ROS topics as HDF5 and per-stream MP4. |
| `odin_ros_driver` | C++ (rclcpp) | Odin1 driver with relocalization-map support. |
| `odin_ros_driver_rev1` | C++ (rclcpp) | Odin Lite / SDK2 driver for SLAM cloud and odometry. |
| `pico_odin` | C++ (rclcpp) | Selects an Odin driver and calibrates/corrects the Odin Lite pelvis pose. |

## Prerequisites

- Root `.venv` and ROS 2 Humble SDK; see [installation](docs/install-guide.md)
- Linux x86_64
- `adb` on `PATH`
- `tmux` on `PATH` for the one-command Tianji launcher
- A PICO 4U running `pico_wholebody_stream.apk`
- Two shoe-mounted IMU900 devices for foot fusion (optional)

## Build

```bash
# From the monorepo root after installing its .venv and ROS SDK:
bash tracking/scripts/build.sh --all
cd tracking
source scripts/environment.sh
```

For rebuilding and testing from `tracking/`:

```bash
bash scripts/build.sh --all
colcon test --base-paths src --event-handlers console_direct+
```

For the full setup guide, including ARM64/RK3588, ADB, and smoke tests, see
[`docs/install-guide.md`](docs/install-guide.md).

## Low-level and optional services

The quick-start scripts above are the default PICO arm workflow. Use the
following lower-level commands only when starting individual optional services
or debugging the bridge directly:

```bash
# Terminal 1 — PICO bridge
source scripts/environment.sh
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py

# Terminal 2 — fisheye cameras (optional)
source scripts/environment.sh
ros2 launch fisheye_camera fisheye_camera.launch.py

# Terminal 3 — data collector
source scripts/environment.sh
ros2 run data_collector data_collector_node \
  --config src/data_collector/config/collect_config.yaml
# Press 's' to start, 'd' to stop + save, 'q' to quit
```

## Run with Odin odometry

The Odin launch helper publishes both implementations to the recorder topics
`/raw/odom/odin` and `/raw/odom/odin_highfreq`. Odin1 also provides
`/raw/odom/odin_local` when local pose is enabled.

```bash
# Odin1, optionally with a relocalization map
ros2 launch pico_odin odin_select.launch.py \
  odin_impl:=original \
  odin_map:=/path/to/map_merged.bin

# Odin Lite / SDK2
ros2 launch pico_odin odin_select.launch.py odin_impl:=lite
```

The `pico_odin` Odin Lite path uses an odometry-only device profile. Raw/SLAM
point clouds, cameras, and the Odin IMU stream remain disabled because pelvis
calibration and correction consume only Odin odometry. This avoids unnecessary
UDP bandwidth and memory use. Use the standalone Odin driver profiles when
point-cloud output is explicitly required.

The Lite driver process uses Fast DDS UDPv4 transport to avoid corrupted
shared-memory segments after an abnormal process kill. If another ROS 2
CLI/node reports `ParticipantEntitiesInfo`, `Fast CDR`, or `Bad alloc`, clean
only the stale Fast DDS segments and restart the affected command:

```bash
fastdds shm clean
```

For a tmux bring-up similar to `catkin_exoskeleton_ws`:

```bash
./scripts/start_pico_odin_collect.sh --odin-impl original --odin-map /path/to/map_merged.bin
./scripts/start_pico_odin_collect.sh --odin-lite
tmux attach -t pico_odin_collect
```

`collect_config.yaml` treats `/raw/odom/odin` and `/raw/odom/odin_highfreq` as
required datasets. `/raw/odom/odin_local` is optional because only Odin1
publishes it when map relocalization or local pose is enabled.

### One-time Odin Lite pelvis mounting calibration

Calibrate the rigid `T_pelvis_odin` transform after installing or moving Odin
Lite. The default file is
`~/.config/pico_tracker/odin_pelvis_extrinsics.yaml`; it is reused across runs.
Use the same ROS domain in every terminal. Start the raw drivers in separate
terminals:

```bash
# All terminals (replace 120 with the domain used by your ROS 2 system)
export ROS_DOMAIN_ID=120
export ROS_LOCALHOST_ONLY=0

# Terminal 1 — raw Odin Lite (disable runtime correction during calibration)
ros2 launch pico_odin odin_select.launch.py \
  odin_impl:=lite \
  enable_pelvis_runtime:=false

# Terminal 2 — PICO bridge
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py

# Terminal 3 — standalone calibrator
ros2 run pico_odin odin_pelvis_calibrator
```

When the calibrator reports ready, press the PICO arm-controller `A` button
(lowercase keyboard `a` remains a fallback). Stand upright and still for two
seconds, lean forward/back through roughly 25--35 degrees, rotate left/right
through roughly 30--60 degrees, then return upright and remain still for at
least three seconds. A validated result
is saved atomically; a rejected attempt does not overwrite the previous file.
Both the PICO pelvis and Odin must be stationary during the two neutral holds.
The file records the spatial transform, motion correlation, quality metrics,
and source topics. Legacy schema 1 and 2 files remain readable and are upgraded
in memory; obsolete timing offsets from those files are deliberately ignored.

For a pelvis-mounted Odin Lite, it is useful to reject physically implausible
solutions during recalibration:

```bash
ros2 run pico_odin odin_pelvis_calibrator --ros-args \
  -p mount_abs_z_max_m:=0.25 \
  -p max_condition_number:=30.0
```

Adjust these limits only when the measured installation requires a larger
offset. Inspect the saved result before enabling runtime correction:

```bash
cat ~/.config/pico_tracker/odin_pelvis_extrinsics.yaml
```

For normal use, the default `auto` setting starts pelvis correction whenever
`odin_impl:=lite` is selected:

```bash
ros2 launch pico_odin odin_select.launch.py odin_impl:=lite
```

After 30 synchronized stationary PICO/Odin samples, corrected outputs become
available:

- `/calibrated/odom/pelvis`
- `/calibrated/odom/pelvis_highfreq`
- `/pico/smpl_odin`
- `/pico/smpl_fused_odin` when foot fusion is active

The runtime takes initial pelvis position and height from PICO, then applies
Odin motion increments. It rigidly moves all 24 joints and leaves all raw
topics unchanged. PICO and Odin device timestamps may use unrelated epochs:
the C++ runtime pairs them on the local steady host-receipt clock, interpolates
the high-frequency Odin stream, and estimates transport receive lag from motion
for the current run only. Timing state is rebuilt at every start and is never
loaded from the mounting YAML. A PICO world reset, discontinuity in either Odin
rate, timestamp rollback, frame change, receive gap, or pose-origin jump clears
only the per-run alignment and pauses corrected output until stationary
alignment is reacquired; it never deletes the saved mounting calibration. Use
`pelvis_extrinsics_file:=/path/to/file.yaml` to override the default file.
Runtime input/output topics and alignment/restart thresholds are launch
arguments; inspect them with
`ros2 launch pico_odin odin_select.launch.py --show-args`.

After starting runtime correction, press PICO `A` once while standing still to
establish the per-run alignment. For a basic in-place rotation check, monitor
`/calibrated/odom/pelvis` and verify that the pelvis X/Y position remains nearly
constant while yaw changes. A large circular X/Y drift indicates an invalid
mounting calibration and should be recalibrated before using the corrected
skeleton.

The project supports both of these runtime configurations:

| Configuration | Main output |
|---|---|
| PICO + Odin Lite | `/pico/smpl_odin` |
| PICO + Odin Lite + dual IMU900 | `/pico/smpl_fused_odin` |

For the dual-IMU configuration, calibrate the Odin mounting transform from
`/pico/smpl` first. Then start the dual IMU900 fusion and Odin runtime together;
the runtime subscribes to `/pico/smpl_fused` and publishes the final
`/pico/smpl_fused_odin` stream. Do not run a second IMU node on either serial
port.

## PICO whole-body data

The whole-body protocol sends 24 `BodyTrackerRole` joints as frame types `0x20`
through `0x37`. `pico_bridge` groups joints with the same timestamp into the
unmodified head-origin stream `/pico/smpl_raw`. After the PICO right-controller
A button is pressed, the ground normalizer waits for 30 stable two-foot frames,
locks the initial foot-sole height, and publishes `/pico/smpl`
(`geometry_msgs/msg/PoseArray`) in frame `pico_ground`. In this canonical stream
the floor is `Z=0`, foot soles are on the floor, and standing pelvis/head heights
are positive. `/pico/smpl` is intentionally silent before A or before a stable
floor lock.

The collector stores the array as `/states/smpl/pose` with shape
`(frames, 24, 7)` and layout `[x, y, z, qx, qy, qz, qw]`. Canonical joint names
are stored as an HDF5 group attribute.

## PICO + dual IMU900 foot fusion

The fusion node preserves non-foot PICO joint positions. For each foot, it uses
the IMU900 orientation and reconstructs position from the PICO ankle plus the
calibrated neutral offset. It publishes `/pico/smpl_fused`; the canonical
ground-aligned `/pico/smpl` remains unchanged by foot fusion.

The default hardware assignment is left foot `/dev/ttyUSB0` and right foot
`/dev/ttyUSB1`. From the repository root, start the PICO bridge first, then
launch both IMU drivers and the fusion node in another terminal:

```bash
# Terminal 1 — PICO bridge
source scripts/environment.sh
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py

# Terminal 2 — dual IMU900 driver and fusion node
source scripts/environment.sh
ros2 launch pico_bridge start_pico_foot_fusion.launch.py \
  left_port:=/dev/ttyUSB0 \
  right_port:=/dev/ttyUSB1
```

Verify both IMUs and the PICO body streams. `/pico/smpl_raw` should appear as
soon as body tracking is active. Press A and stand still before expecting
`/pico/smpl`:

```bash
ros2 topic echo /imu/left_feet/ready --once
ros2 topic echo /imu/right_feet/ready --once
ros2 topic hz /imu/left_feet
ros2 topic hz /imu/right_feet
ros2 topic hz /pico/smpl_raw
ros2 topic hz /pico/smpl
ros2 topic hz /pico/smpl_fused
```

Stand still with both feet flat, then press the PICO right-controller A button.
The automatic transaction resets the left and right IMU900 Z axes, waits one
second, discards old samples, and collects 60 fresh neutral samples.

The node resumes `/pico/smpl_fused` after logging
`PICO foot IMU calibration complete`. A hardware reset or IMU reconnect requires
a new calibration.

The manual fallback below collects the same software calibration samples but
does **not** send the IMU900 hardware Z-axis reset command:

```bash
ros2 service call /pico_foot_imu_fusion/calibrate std_srvs/srv/Trigger "{}"
```

### PICO + Odin Lite + dual IMU900

After the Odin mounting calibration is valid, keep the PICO bridge running and
start the following in separate terminals:

```bash
# Terminal 1 — dual IMU900 fusion
ros2 launch pico_bridge start_pico_foot_fusion.launch.py \
  left_port:=/dev/ttyUSB0 \
  right_port:=/dev/ttyUSB1 \
  imu_reset_settle_sec:=1.0 \
  calibration_samples:=60

# Terminal 2 — Odin Lite runtime correction
ros2 launch pico_odin odin_select.launch.py \
  odin_impl:=lite \
  enable_pelvis_runtime:=true
```

Stand still and press PICO `A` once. This establishes PICO ground alignment,
resets and calibrates both foot IMUs, and initializes the per-run Odin
alignment. Verify the final stream before visualization:

```bash
ros2 topic hz /pico/smpl_fused_odin
```

To visualize the final fused skeleton while overlaying the Odin-only skeleton:

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_fused_odin \
  --show-raw \
  --raw-topic /pico/smpl_odin \
  --scale 1.0 \
  --rate 60 \
  --timeout 2.0
```

The vendored IM900/IM948 driver uses separate 115200-baud serial ports, publishes
at approximately 110 Hz, and adds ready-state topics and automatic reconnects.
The validated default foot mount quaternion is `[0, 0, 0, 1]`.

See [`docs/PICO_FOOT_IMU_FUSION.md`](docs/PICO_FOOT_IMU_FUSION.md) for mounting,
calibration, and troubleshooting details.

## MuJoCo geometric skeleton viewer

The optional MuJoCo viewer renders 24 joint spheres, 23 capsule bones, and
oriented foot boxes. To display the canonical ground-aligned PICO skeleton:

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0
```

To compare IMU-corrected and original foot poses in the same coordinate frame:

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_fused \
  --show-raw \
  --raw-topic /pico/smpl \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0
```

To display the raw PICO HMD and both controller poses alongside the skeleton,
add `--show-controllers`. This is an explicit overlay independent of
`--show-raw`, so a raw skeleton comparison does not add extra endpoint markers.
With `--raw-offset 0 0 0`, the raw and corrected
skeletons are coincident and the foreground skeleton can hide the raw one; use
a small display-only offset when comparing them:

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_palm_corrected \
  --show-raw --raw-topic /pico/smpl_raw \
  --palm-topic /pico/palm_left \
  --right-palm-topic /pico/palm_right \
  --show-arm-axes --show-controllers \
  --raw-offset 0.30 0 0
```

`--show-arm-axes` applies only to the primary corrected/TCP skeleton. The raw
PICO overlay is intentionally drawn without local arm XYZ triads; add
`--show-raw-arm-axes` only when those raw diagnostics are explicitly needed.

The three pose topics are `/pico/pose/head`, `/pico/pose/left_hand`, and
`/pico/pose/right_hand`; override them with `--head-topic`,
`--left-controller-topic`, or `--right-controller-topic` when needed. These
poses use the same PICO-world display transform as the skeleton and palm
markers, and are visualization-only.

To inspect the PICO upper body and arm endpoints for tape-measure validation,
use the hands-only mode. It renders the upper body from the pelvis through the
head and both elbow-wrist-hand chains, hides the legs, keeps the ground and
world axes, and overlays the left/right hand endpoint coordinates in meters in
the `pico_ground` frame:

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl \
  --hands-only \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0
```

Hands-only mode automatically adds red/green/blue local X/Y/Z triads only at
the left and right shoulder, elbow, wrist, and hand keypoints. Spine, pelvis,
and head keypoints do not receive local axes; the fixed world axes remain
visible. For the full-body view, `--show-arm-axes` enables the same arm-only
triads without hiding the legs.

The same mode can be used with `/pico/smpl_odin` or
`/pico/smpl_fused_odin` when evaluating Odin-corrected hand positions.

### Palm TCP calibration and palm-constrained skeleton

The recommended entry point is the unified menu. It supports arbitrary
left/right single-item calibration, ordered full-side adaptation, and read-only
artifact status inspection:

```bash
./scripts/calibrate_pico_arm.sh
./scripts/calibrate_pico_arm.sh status
```

TCP calibration is a one-time operation for each controller/glove mounting.
Re-run it only after the mounting changes. To calibrate the left glove palm
center, keep the palm center fixed at one point, change the controller
orientation between poses, and press the keyboard space bar four times. These
four samples solve only the controller-to-palm translation. After the position
gate passes, extend both arms forward and horizontal with the palms facing each
other, then press SPACE a fifth time and hold the pose for about 1--2 seconds.
The selected side uses at least 120 time-paired samples and an SO(3) robust
mean. Palm roll/pitch are gravity-level while yaw follows the HMD heading. The
artifact is saved only after both stages pass:

For direct non-menu operation, select the side and item explicitly:

```bash
./scripts/calibrate_pico_arm.sh left tcp
./scripts/calibrate_pico_arm.sh right tcp
```

The calibration artifacts are:

```text
left:  ~/.config/pico_tracker/pico_left_palm_tcp.yaml
right: ~/.config/pico_tracker/pico_right_palm_tcp.yaml
```

If a new PICO session leaves one palm orientation visibly biased, keep the
existing TCP translation, wrist pivot, and arm lengths and recalibrate only
that side's TCP orientation. In the project `source scripts/environment.sh`, use:

```bash
./scripts/calibrate_pico_palm_orientation.sh left
# or
./scripts/calibrate_pico_palm_orientation.sh right
```

Stand upright with both arms straight forward and horizontal, palms facing
each other, then press Space and hold still. The selected running TCP publisher
also exposes `/pico/palm_orientation/<side>/calibrate` (`std_srvs/Trigger`) for
future coordinated exoskeleton calibration. A successful update is applied
immediately without restarting the publisher. It changes only
`quaternion_xyzw`; TCP translation, wrist calibration, and arm geometry remain
valid. Failure leaves both disk and runtime state unchanged.

For normal runtime, do not keep either interactive calibrator running. After
the PICO bridge is publishing `/pico/smpl_raw` and both raw controller poses,
one launch command starts the left and right read-only TCP publishers plus the
palm-constrained skeleton filter:

### Bilateral individual arm-geometry calibration

When the default PICO SMPL upper/forearm proportions distort the
palm-constrained skeleton, calibrate each side independently. Keep the PICO
driver online and run the selected side from the root .venv / ROS SDK environment:

```bash
./scripts/calibrate_pico_arm.sh left geometry
./scripts/calibrate_pico_arm.sh right geometry
```

Both sides share the solver code but are captured, validated, and stored
separately; never mirror one side's measured lengths into the other side.

The launch starts the read-only left palm publisher and begins automatically
after preflight. Follow the terminal prompts: neutral arm, two static straight
forward reaches, a static upper-arm-down approximately 90-degree elbow pose,
an independent straight-reach validation, and neutral return. Reach each pose
before its countdown ends; the longest sufficiently static suffix is selected.
Neutral, straight, and approximately 90-degree wrist positions eliminate the
unknown shoulder anchor: straight-minus-right-angle estimates upper-arm length,
and right-angle-minus-neutral estimates forearm length. The closest two of the
three straight captures are selected and the outlier is recorded. Raw PICO
shoulder/elbow/wrist angle error is retained as diagnostic evidence only; it
neither defines bone length nor vetoes independent-palm geometry that passes
the structural gates. Evidence and the fail-closed `pico_left_arm_geometry_quick_v3`
candidate are written under `recordings/pico_left_arm_geometry_<timestamp>/`.
The NPZ preserves raw palm position/orientation, raw PICO wrist, and derived
wrist so a rejected run remains fully auditable. Legacy v1/v2 candidates are
not loadable.

Only candidates whose `gate_report.json` has `valid=true` and whose YAML has
`candidate_status: accepted` are atomically activated under
`~/.config/pico_tracker/`. Rejected captures stay in `recordings/` for audit and
never overwrite the active artifact. Inspect active state with:

```bash
./scripts/calibrate_pico_arm.sh status
./scripts/start_pico_m0.sh --viewer --record --duration 120
```

Both status sides must report `geometry_source=quick_arm_artifact` with their
own positive `geometry_revision`. A deliberately uncalibrated side may remain
`raw_smpl_baseline` with revision zero, but that is not bilateral closeout.

The command prints the exact NPZ capture path. Produce the fail-closed report:

```bash
ros2 run pico_bridge pico_m0_comparison_report report \
  /absolute/path/pico_m0_capture.npz \
  --output /absolute/path/pico_m0_gate.json
```

```bash
ros2 launch pico_bridge start_pico_palm_skeleton_filter.launch.py \
  left_tcp_artifact:="$HOME/.config/pico_tracker/pico_left_palm_tcp.yaml" \
  right_tcp_artifact:="$HOME/.config/pico_tracker/pico_right_palm_tcp.yaml" \
  left_wrist_pivot_artifact:="$HOME/.config/pico_tracker/pico_left_wrist_pivot.yaml" \
  right_wrist_pivot_artifact:="$HOME/.config/pico_tracker/pico_right_wrist_pivot.yaml" \
  require_wrist_pivot_artifact:=true
```

This publishes `/pico/palm_left`, `/pico/palm_right`, the unchanged
PICO-frame skeleton `/pico/smpl_palm_corrected`, and
`/pico/smpl_palm_corrected_ik`.  The IK topic has exactly the same joint
positions. Its shoulder frame bases are adapted by local-X right
multiplication (`left=+pi/2`, `right=-pi/2`). Each elbow frame is rebuilt so
local X points from elbow to wrist, local Y is the elbow flexion axis selected
from the shoulder-elbow-wrist plane, and local Z completes a right-handed
frame. Both elbow local +Y axes use the body-left hemisphere. A straight or
nearly straight arm uses body-left projected onto the plane normal to the
forearm because the flexion plane is then unobservable. If body-left cannot
reliably select the sign of an observable flexion axis, the last valid elbow-Y
sign preserves temporal continuity. A degenerate IK frame is skipped and
reported through `ik_frame_valid`/`ik_frame_failure_reason`; it never interrupts
the unchanged corrected-skeleton stream. Verify the runtime chain with:

```bash
ros2 topic hz /pico/palm_left
ros2 topic hz /pico/palm_right
ros2 topic hz /pico/smpl_palm_corrected
ros2 topic hz /pico/smpl_palm_corrected_ik
ros2 topic echo /pico/smpl_palm_corrected/status --once
```

`./scripts/start_pico_m0.sh --viewer` displays the IK-frame topic while keeping
the raw SMPL overlay for comparison. This changes the rendered shoulder and
elbow axes, not the skeleton geometry or palm targets.

The runtime publishers load the artifacts read-only and apply each calibrated
`T_controller_palm` exactly once to its raw controller pose. They never modify
the calibration files and require no keyboard input. If you intentionally run
manual palm publishers or calibrators, pass `start_palm_publishers:=false` to
the launch command to prevent duplicate publishers on the palm topics.

The viewer can also fall back to the saved TCP files
`--left-tcp-artifact`/`--right-tcp-artifact` and raw controller topics.
When `--left-wrist-pivot-artifact` and `--right-wrist-pivot-artifact` are
configured, the viewer always reconstructs the wrist marker from the same TCP
palm sample by moving along palm-local negative X. Legacy `wrist_to_palm_m`
artifacts contribute only their vector norm; their old XYZ direction is not
used. This keeps the wrist orientation identical to the palm and avoids
an independently published wrist topic (with a different timestamp or
calibration) overwriting the TCP geometry. Without a pivot artifact, a live
wrist topic is used, then the corrected PoseArray wrist joints. The overlay
status reports `wrist_pivot`, `topic`, or `primary_posearray` as the source.

See [`docs/PICO_PALM_SKELETON_FILTER.md`](docs/PICO_PALM_SKELETON_FILTER.md)
for the complete data-flow, status fields, and troubleshooting guide.

The fused skeleton uses blue joints, gray bones, and orange foot boxes. The
unfused canonical PICO skeleton is thinner and transparent green. Both streams
use the same `pico_ground` coordinates; missing or stale overlay data never
blocks fused output. `/pico/smpl_raw` is the separate APK/head-origin diagnostic
stream and must not be overlaid without applying its ground offset.

The viewer subscribes with `BEST_EFFORT` QoS. A stale primary stream remains
visible in gray and produces a throttled terminal warning. MuJoCo is installed
in the root `.venv`.

## Testing without a real PICO

```bash
python3 src/pico_bridge/scripts/mock_pico_server.py --fps 30 --duration 60
```

## Layout details

See [`docs/workspace-layout.md`](docs/workspace-layout.md).

## Design and history

- [`docs/PICO_Streaming_Guide.md`](docs/PICO_Streaming_Guide.md) — PICO protocol reference.
- [`docs/superpowers/specs/`](docs/superpowers/specs/) — design documents.
- [`docs/superpowers/plans/`](docs/superpowers/plans/) — implementation plans.
- [`reference/receiver.py`](reference/receiver.py) — original Python receiver.
