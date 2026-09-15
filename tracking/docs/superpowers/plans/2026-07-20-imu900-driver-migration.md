# Original IMU900 Driver Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the incorrectly ported `LL/RL` lower-controller driver with the reference `imu_ros2` IM900/IM948 driver plus narrowly scoped ready/reconnect hardening, then connect two standalone foot IMU900 devices to the existing PICO fusion node.

**Architecture:** Vendor `/home/zj/sdk_test/catkin_exoskeleton_ws/src/imu_ros2` as `src/imu_ros2`, preserving its protocol and message behavior while adding only per-channel ready/reconnect handling. Keep all PICO-specific ports, topic remaps, mounting defaults, and fusion rules in `pico_bridge`, then delete `pico_imu900_driver` and its custom status-message coupling.

**Tech Stack:** ROS 2 Humble, C++17, `rclcpp`, `sensor_msgs/msg/Imu`, ROS 2 launch, YAML, ament/colcon, Pixi, Python unittest/pytest.

## Global Constraints

- `src/imu_ros2` must preserve the reference protocol and message behavior; the only permitted local additions are per-channel ready status and serial reconnect handling for the foot runtime.
- Each IMU900 uses its own serial port; defaults are `/dev/ttyUSB0` for the left foot and `/dev/ttyUSB1` for the right foot.
- Driver settings are `baudrate=115200`, `report_hz=110`, `report_tag=46`, `target_address=255`, compass enabled, device timestamps enabled, and quaternion continuity enabled.
- `/imu/left_feet` and `/imu/right_feet` remain the fusion input topics.
- The new IMU900-to-foot mounting correction defaults to the currently measured cyclic-axis candidate `[0.5, 0.5, 0.5, 0.5]` for both feet; keep both sides independently overrideable.
- Fusion requires both driver ready topics plus valid, fresh messages from both IMUs.
- Preserve the user's untracked `.deb` file and `pc_stream_records/` directory.
- Do not modify any file under `/home/zj/sdk_test/catkin_exoskeleton_ws`.

## File Structure

- Create `src/imu_ros2/**`: reference driver package, including protocol library, nodes, launches, tests, ready topics, and reconnect handling.
- Create `src/pico_bridge/config/imu900_feet.yaml`: PICO-specific two-foot IMU900 shared parameters and semantic channel names.
- Create `src/pico_bridge/launch/start_pico_foot_fusion.launch.py`: configurable two-port IMU900 driver plus fusion-node launch.
- Create `src/pico_bridge/test/test_imu900_foot_integration.py`: static integration contract for configuration, launch defaults, and removal of the wrong driver dependency.
- Create `src/pico_bridge/include/pico_bridge/foot_imu_input_state.hpp`: testable IMU validity/freshness cache and finite-position validation.
- Create `src/pico_bridge/test/test_foot_imu_input_state.cpp`: invalidation, expiry, and position-validity behavior tests.
- Modify `src/pico_bridge/src/pico_foot_imu_fusion_node.cpp`: gate standard IMU validity/freshness with the new per-channel ready topics.
- Modify `src/pico_bridge/CMakeLists.txt` and `src/pico_bridge/package.xml`: install/test the integration files and depend on `imu_ros2` only at runtime.
- Delete `src/pico_imu900_driver/**`: remove the incorrectly named legacy controller driver.
- Modify `pixi.toml`, `README.md`, and `docs/PICO_FOOT_IMU_FUSION.md`: build/test and operating instructions for the real driver.

---

### Task 1: Vendor the reference IM900/IM948 driver and add foot-runtime hardening

**Files:**
- Create: `src/imu_ros2/**`
- Source: `/home/zj/sdk_test/catkin_exoskeleton_ws/src/imu_ros2/**`

**Interfaces:**
- Produces: executables `imu_ros2/imu_node` and `imu_ros2/imu_multi_node`.
- Produces: per-channel topic `<channel_name>/imuData_raw` with type `sensor_msgs/msg/Imu`.
- Produces: per-channel services `zero_z_axis`, `clear_world_axes`, and `restore_world_axes`.

- [ ] **Step 1: Verify the destination is absent**

Run:

```bash
test ! -e src/imu_ros2
```

Expected: exit code 0.

- [ ] **Step 2: Copy the complete package before applying the narrow runtime hardening patch**

Run this mechanical source import from the repository root:

```bash
cp -a /home/zj/sdk_test/catkin_exoskeleton_ws/src/imu_ros2 src/imu_ros2
```

- [ ] **Step 3: Prove the imported protocol and message implementation matches the reference, allowing only the documented ready/reconnect additions**

Run:

```bash
diff -qr /home/zj/sdk_test/catkin_exoskeleton_ws/src/imu_ros2 src/imu_ros2
```

Expected: no output and exit code 0.

- [ ] **Step 4: Build and test the imported package**

Run:

```bash
pixi run bash -lc 'export ROS_VERSION=2; colcon build --base-paths src --symlink-install --packages-select imu_ros2 --cmake-args -DPython_ROOT_DIR=$CONDA_PREFIX -DPython_FIND_VIRTUALENV=ONLY'
pixi run bash -lc 'export ROS_VERSION=2; colcon test --base-paths src --packages-select imu_ros2 --event-handlers console_direct+'
pixi run bash -lc 'colcon test-result --test-result-base build/imu_ros2 --verbose'
```

Expected: package builds; `test_im948_protocol` passes with zero failures.

- [ ] **Step 5: Commit the exact import**

```bash
git add src/imu_ros2
git commit -m "feat: vendor original IMU900 ROS2 driver"
```

### Task 2: Add the PICO-specific dual-foot launch configuration

**Files:**
- Create: `src/pico_bridge/config/imu900_feet.yaml`
- Create: `src/pico_bridge/launch/start_pico_foot_fusion.launch.py`
- Create: `src/pico_bridge/test/test_imu900_foot_integration.py`
- Modify: `src/pico_bridge/CMakeLists.txt`
- Modify: `src/pico_bridge/package.xml`

**Interfaces:**
- Consumes: `imu_ros2/imu_multi_node`.
- Produces: launch arguments `left_port`, `right_port`, `max_imu_age_sec`, and `calibration_samples`.
- Produces: `/imu/left_feet`, `/imu/right_feet`, and the `pico_foot_imu_fusion` node.

- [ ] **Step 1: Write the failing integration configuration test**

Create a unittest that loads the YAML and launch module:

```python
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import unittest
import yaml


ROOT = Path(__file__).parents[1]


class Imu900FootIntegrationTest(unittest.TestCase):
    def test_driver_parameters_match_reference_profile(self):
        config = yaml.safe_load((ROOT / "config/imu900_feet.yaml").read_text())
        params = config["im900_foot_multi_node"]["ros__parameters"]
        self.assertEqual(params["channel_names"], ["im900/left_foot", "im900/right_foot"])
        self.assertEqual(params["frame_ids"], ["left_foot_imu_link", "right_foot_imu_link"])
        self.assertEqual(params["baudrate"], 115200)
        self.assertEqual(params["report_hz"], 110)
        self.assertEqual(params["report_tag"], 46)
        self.assertEqual(params["target_address"], 255)
        self.assertTrue(params["enable_compass"])
        self.assertTrue(params["use_device_timestamp"])
        self.assertTrue(params["use_quaternion_continuity"])
        self.assertFalse(params["clear_world_axes"])
        self.assertFalse(params["restore_world_axes"])

    def test_launch_defaults_and_remaps_are_explicit(self):
        path = ROOT / "launch/start_pico_foot_fusion.launch.py"
        spec = spec_from_file_location("start_pico_foot_fusion", path)
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(module.DEFAULT_LEFT_PORT, "/dev/ttyUSB0")
        self.assertEqual(module.DEFAULT_RIGHT_PORT, "/dev/ttyUSB1")
        self.assertEqual(module.IMU_REMAPPINGS, [
            ("im900/left_foot/imuData_raw", "/imu/left_feet"),
            ("im900/right_foot/imuData_raw", "/imu/right_feet"),
        ])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Register and run the test to verify RED**

Add this inside `if(BUILD_TESTING)` in `src/pico_bridge/CMakeLists.txt`:

```cmake
ament_add_pytest_test(test_imu900_foot_integration test/test_imu900_foot_integration.py)
```

Run:

```bash
pixi run python -m unittest src/pico_bridge/test/test_imu900_foot_integration.py -v
```

Expected: FAIL because `imu900_feet.yaml` and `start_pico_foot_fusion.launch.py` do not exist.

- [ ] **Step 3: Add the exact shared IMU900 configuration**

Create `src/pico_bridge/config/imu900_feet.yaml`:

```yaml
im900_foot_multi_node:
  ros__parameters:
    channel_names: ["im900/left_foot", "im900/right_foot"]
    frame_ids: ["left_foot_imu_link", "right_foot_imu_link"]
    baudrate: 115200
    report_hz: 110
    report_tag: 46
    battery_query_period_ms: 5000
    enable_compass: true
    use_reliable_qos: false
    use_device_timestamp: true
    coalesce_frames_per_poll: true
    align_quaternion_hemisphere: true
    force_positive_w: true
    use_quaternion_continuity: true
    clear_ins_position: true
    clear_world_axes: false
    restore_world_axes: false
    target_address: 255
    imu_topic: "imuData_raw"
    battery_topic: "battery"
    log_magnetic_field_status: false
```

- [ ] **Step 4: Add the combined driver and fusion launch**

Create `start_pico_foot_fusion.launch.py` with module constants from the test. Declare
`left_port`, `right_port`, `max_imu_age_sec`, and `calibration_samples`; resolve the installed
YAML using `get_package_share_directory("pico_bridge")`; start `imu_multi_node` as
`im900_foot_multi_node` with the YAML plus a `ports` override; apply `IMU_REMAPPINGS`; then
start `pico_foot_imu_fusion` with identity `[0.0, 0.0, 0.0, 1.0]` left/right mounting
quaternions and the two fusion launch arguments.

The node-specific launch configuration must contain:

```python
DEFAULT_LEFT_PORT = "/dev/ttyUSB0"
DEFAULT_RIGHT_PORT = "/dev/ttyUSB1"
IMU_REMAPPINGS = [
    ("im900/left_foot/imuData_raw", "/imu/left_feet"),
    ("im900/right_foot/imuData_raw", "/imu/right_feet"),
]
```

- [ ] **Step 5: Install config and declare the runtime dependency**

Change the directory install in `CMakeLists.txt` to:

```cmake
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
```

Add to `package.xml`:

```xml
<exec_depend>imu_ros2</exec_depend>
<exec_depend>ament_index_python</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>ros2launch</exec_depend>
<test_depend>python3-yaml</test_depend>
```

- [ ] **Step 6: Run the new test and build**

```bash
pixi run python -m unittest src/pico_bridge/test/test_imu900_foot_integration.py -v
pixi run bash -lc 'export ROS_VERSION=2; colcon build --base-paths src --symlink-install --packages-select imu_ros2 pico_bridge --cmake-args -DPython_ROOT_DIR=$CONDA_PREFIX -DPython_FIND_VIRTUALENV=ONLY'
```

Expected: two integration tests pass and both packages build.

- [ ] **Step 7: Commit the integration launch**

```bash
git add src/pico_bridge/config/imu900_feet.yaml src/pico_bridge/launch/start_pico_foot_fusion.launch.py src/pico_bridge/test/test_imu900_foot_integration.py src/pico_bridge/CMakeLists.txt src/pico_bridge/package.xml
git commit -m "feat: launch dual foot IMU900 devices"
```

### Task 3: Decouple fusion readiness from the incorrect driver

**Files:**
- Modify: `src/pico_bridge/test/test_imu900_foot_integration.py`
- Modify: `src/pico_bridge/src/pico_foot_imu_fusion_node.cpp`
- Modify: `src/pico_bridge/CMakeLists.txt`
- Modify: `src/pico_bridge/package.xml`
- Create: `src/pico_bridge/include/pico_bridge/foot_imu_input_state.hpp`
- Create: `src/pico_bridge/test/test_foot_imu_input_state.cpp`

**Interfaces:**
- Consumes: valid, fresh `sensor_msgs/msg/Imu` messages on both input topics.
- Produces: unchanged calibration/reset services and `/pico/smpl_fused` output behavior.

- [ ] **Step 1: Extend the contract test to verify standard-message readiness**

Add:

```python
    def test_fusion_has_no_legacy_driver_dependency(self):
        source = (ROOT / "src/pico_foot_imu_fusion_node.cpp").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()
        package = (ROOT / "package.xml").read_text()
        for text in (source, cmake, package):
            self.assertNotIn("pico_imu900_driver", text)
            self.assertNotIn("DriverStatus", text)
        self.assertIn("orientation_covariance[0] < 0.0", source)
```

- [ ] **Step 2: Run the test to verify RED**

```bash
pixi run python -m unittest src/pico_bridge/test/test_imu900_foot_integration.py -v
```

Expected: FAIL because the source and manifests still depend on `pico_imu900_driver`.

- [ ] **Step 3: Remove status-message readiness and reject unavailable orientation**

In `pico_foot_imu_fusion_node.cpp`:

- remove the legacy `DriverStatus` include, subscription, `require_driver_ready` parameter, and
  `driver_ready_` state; use the two standard ready topics instead.
- before normalizing an IMU quaternion, reject the standard ROS unavailable marker and clear that
  side's cached input immediately:

```cpp
if (msg.orientation_covariance[0] < 0.0) {
  warn("ignoring IMU message whose orientation is unavailable");
  return;
}
```

- make calibration and publishing readiness depend only on both cached IMUs, their receive-time
  freshness, a valid PICO frame, and an existing baseline;
- reject PICO frames if any of the 24 positions contains NaN or infinity;
- retain independently configurable defaults `[0.5,0.5,0.5,0.5]` for `left_mount_quaternion` and `right_mount_quaternion`.

In `CMakeLists.txt`, remove `find_package(pico_imu900_driver REQUIRED)` and remove it from
`ament_target_dependencies`. In `package.xml`, remove `<depend>pico_imu900_driver</depend>`.

- [ ] **Step 4: Run focused tests and build**

```bash
pixi run python -m unittest src/pico_bridge/test/test_imu900_foot_integration.py -v
pixi run bash -lc 'export ROS_VERSION=2; colcon build --base-paths src --symlink-install --packages-select imu_ros2 pico_bridge --cmake-args -DPython_ROOT_DIR=$CONDA_PREFIX -DPython_FIND_VIRTUALENV=ONLY'
pixi run bash -lc 'export ROS_VERSION=2; colcon test --base-paths src --packages-select pico_bridge --event-handlers console_direct+'
```

Expected: integration and existing fusion tests pass; `pico_bridge` builds without the old package.

- [ ] **Step 5: Commit the readiness change**

```bash
git add src/pico_bridge
git commit -m "fix: derive foot fusion readiness from IMU data"
```

### Task 4: Remove the wrong driver and update repository workflows

**Files:**
- Delete: `src/pico_imu900_driver/**`
- Modify: `src/pico_bridge/test/test_imu900_foot_integration.py`
- Modify: `pixi.toml`
- Modify: `README.md`
- Modify: `docs/PICO_FOOT_IMU_FUSION.md`

**Interfaces:**
- Produces: `pixi run build-core`, `pixi run build`, and `pixi run test` selecting `imu_ros2`.
- Produces: operator commands using `pico_bridge start_pico_foot_fusion.launch.py`.

- [ ] **Step 1: Extend the migration test to cover repository cleanup**

Add:

```python
    def test_wrong_driver_is_removed_from_build_workflow(self):
        repository = ROOT.parents[1]
        self.assertFalse((repository / "src/pico_imu900_driver").exists())
        pixi = (repository / "pixi.toml").read_text()
        self.assertNotIn("pico_imu900_driver", pixi)
        self.assertIn("imu_ros2", pixi)
```

- [ ] **Step 2: Run the test to verify RED**

```bash
pixi run python -m unittest src/pico_bridge/test/test_imu900_foot_integration.py -v
```

Expected: FAIL because `src/pico_imu900_driver` still exists and Pixi still selects it.

- [ ] **Step 3: Remove the incorrect package and switch Pixi package lists**

Delete only the tracked `src/pico_imu900_driver` tree. Replace every
`pico_imu900_driver` selection in `pixi.toml` with `imu_ros2`; preserve all other selected
packages and task commands.

- [ ] **Step 4: Update operating documentation**

Document this startup sequence in `README.md` and `docs/PICO_FOOT_IMU_FUSION.md`:

```bash
pixi run build
source install/setup.bash
ros2 launch pico_bridge start_pico_foot_fusion.launch.py \
  left_port:=/dev/ttyUSB0 \
  right_port:=/dev/ttyUSB1
ros2 topic hz /imu/left_feet
ros2 topic hz /imu/right_feet
ros2 service call /pico_foot_imu_fusion/calibrate std_srvs/srv/Trigger "{}"
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_fused --scale 1.0 --rate 60 --timeout 1.0
```

State explicitly that the two devices use separate ports, that the default mounting correction is
the measured cyclic-axis candidate `[0.5,0.5,0.5,0.5]`,
and that users should swap the launch port arguments if left/right motion is reversed.

- [ ] **Step 5: Run cleanup test and repository search**

```bash
pixi run python -m unittest src/pico_bridge/test/test_imu900_foot_integration.py -v
rg -n "pico_imu900_driver|DriverStatus|imu_ids: \[7, 6\]|imu_tags: \[\"LL\", \"RL\"\]" README.md docs/PICO_FOOT_IMU_FUSION.md pixi.toml src
```

Expected: tests pass; `rg` returns no matches in active code/config/docs.

- [ ] **Step 6: Commit cleanup and documentation**

```bash
git add -A src/pico_imu900_driver src/pico_bridge/test/test_imu900_foot_integration.py pixi.toml README.md docs/PICO_FOOT_IMU_FUSION.md
git commit -m "chore: remove legacy foot IMU driver"
```

### Task 5: Full verification and hardware handoff

**Files:**
- Verify only; do not edit reference workspace or user data.

**Interfaces:**
- Verifies the complete source, build, test, and launch contract.

- [ ] **Step 1: Recheck the vendored source identity**

```bash
diff -qr /home/zj/sdk_test/catkin_exoskeleton_ws/src/imu_ros2 src/imu_ros2
```

Expected: no output.

- [ ] **Step 2: Run all project builds and tests**

```bash
pixi run build
pixi run test
pixi run bash -lc 'colcon test-result --test-result-base build --verbose'
```

Expected: all selected packages build and all tests report zero failures.

- [ ] **Step 3: Inspect the resulting launch description without hardware**

```bash
pixi run bash -lc 'source install/setup.bash && ros2 launch pico_bridge start_pico_foot_fusion.launch.py --show-args'
```

Expected: output includes both default `/dev/ttyUSB0` and `/dev/ttyUSB1` ports, `max_imu_age_sec`, and
`calibration_samples`.

- [ ] **Step 4: Record hardware validation as pending unless devices are connected**

If `/dev/ttyUSB0` and `/dev/ttyUSB1` exist, run the launch and verify both topic
rates are near 110 Hz and quaternion norms are near 1 before calibration. Otherwise report that
software verification passed but real-device direction and reconnect behavior remain to be tested.

- [ ] **Step 5: Confirm worktree scope**

```bash
git status --short
git log --oneline --decorate -6
```

Expected: only the user's pre-existing untracked `.deb` and `pc_stream_records/` remain outside
the implementation commits.
