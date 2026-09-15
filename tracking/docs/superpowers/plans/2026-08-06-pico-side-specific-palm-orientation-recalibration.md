# PICO Side-Specific Palm Orientation Recalibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a left/right independent palm-orientation recalibration that updates only the selected TCP quaternion, applies immediately to the running publisher, exposes a ROS service, and preserves existing arm-geometry validity.

**Architecture:** Pure Python core code owns SO(3) averaging, semantic translation lineage, and atomic artifact mutation. Each side-specific TCP publisher owns the active transform and exposes a `std_srvs/Trigger` calibration service. A small shell script prompts the operator and calls the same service used by future exoskeleton calibration.

**Tech Stack:** ROS 2 Humble/rclpy, Python 3.11, NumPy, YAML, Bash, pytest/colcon.

## Global Constraints

- Reference pose: both arms forward and horizontal, palms facing each other, head facing forward.
- One request updates exactly one side.
- Preserve `translation_m`, wrist artifact, and arm lengths byte-for-value.
- Orientation-only changes must not invalidate a valid geometry artifact.
- Translation changes must invalidate geometry.
- Do not use raw SMPL wrist orientation as truth.
- Default `ROS_DOMAIN_ID=120`, `ROS_LOCALHOST_ONLY=1`.
- Never add `recordings/` data to Git.
- No local commit is created during this execution.

---

### Task 1: Pure SO(3) and TCP Orientation Mutation Core

**Files:**
- Create: `src/pico_bridge/scripts/pico_palm_orientation_core.py`
- Create: `src/pico_bridge/test/test_pico_palm_orientation_core.py`

**Interfaces:**
- Produces `OrientationSample(controller_rotation, head_rotation, stamp_ns, tracking_epoch)`.
- Produces `solve_orientation(samples, current_rotation, gates) -> OrientationSolution`.
- Produces `translation_fingerprint(document) -> str`.
- Produces `build_orientation_only_update(document, solution, old_sha256) -> dict`.
- Produces `atomic_replace_if_unchanged(path, old_sha256, document) -> None`.

- [ ] Write tests for exact recovery, quaternion-sign invariance, Huber outlier rejection, insufficient samples, excessive RMS/correction, unchanged translation, revision updates, PSD covariance, concurrent-file rejection, and atomic replacement.
- [ ] Run the focused test and verify failures are caused by the missing module.
- [ ] Implement the minimal pure module with right-local residuals and canonical translation hashing.
- [ ] Run focused tests and verify all pass.

### Task 2: Translation-Semantic Geometry Lineage

**Files:**
- Modify: `src/pico_bridge/scripts/pico_calibration_artifact.py`
- Modify: `src/pico_bridge/scripts/pico_arm_geometry_calibrator.py`
- Modify: `src/pico_bridge/scripts/pico_palm_tcp_calibrator.py`
- Modify: `src/pico_bridge/test/test_pico_calibration_artifact.py`
- Modify: `src/pico_bridge/test/test_pico_arm_geometry_calibrator.py`

**Interfaces:**
- TCP validation exposes `translation_revision`, `translation_fingerprint_sha256`, and orientation-only ancestor hashes.
- New geometry candidates store `tcp_translation_revision` and `tcp_translation_fingerprint_sha256`.
- Legacy geometry accepts the current full TCP hash or a controlled orientation-only ancestor hash, while still comparing its legacy revision to `translation_revision`.

- [ ] Add failing tests proving orientation-only TCP mutation preserves both new and legacy geometry validity while translation mutation fails.
- [ ] Run focused tests and observe lineage mismatch failures.
- [ ] Extend TCP artifacts and validators with semantic translation lineage and legacy migration.
- [ ] Update geometry generation and validation to consume semantic lineage.
- [ ] Run focused tests and verify all pass.

### Task 3: Side-Specific Publisher Calibration Service and Live Swap

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_tcp_publisher.py`
- Modify: `src/pico_bridge/scripts/pico_palm_tcp_runtime.py`
- Modify: `src/pico_bridge/test/test_pico_palm_tcp_runtime.py`
- Modify: `src/pico_bridge/test/test_pico_palm_orientation_service.py`
- Modify: `src/pico_bridge/CMakeLists.txt`

**Interfaces:**
- Each publisher exposes `/pico/palm_orientation/<side>/calibrate` as `std_srvs/srv/Trigger`.
- Service captures at least 120 paired samples, validates skew/epoch/RMS/correction, atomically writes the artifact, then swaps one complete `TcpTransform` under a lock.
- Trigger response message is compact JSON with side, revisions, epoch, sample count, RMS, correction angle, and path.

- [ ] Add failing tests for topic pairing, epoch changes, busy rejection, selected-side isolation, failed-write rollback, and immediate next-frame use of the new transform.
- [ ] Run focused tests and verify the service behavior is absent.
- [ ] Add the HMD/epoch subscriptions, bounded paired buffer, service state, and `MultiThreadedExecutor`.
- [ ] Ensure publisher output continues during capture and swaps only after successful write.
- [ ] Run focused tests and verify all pass.

### Task 4: Standalone Single-Side Script

**Files:**
- Create: `scripts/calibrate_pico_palm_orientation.sh`
- Create: `src/pico_bridge/scripts/pico_palm_orientation_client.py`
- Modify: `src/pico_bridge/CMakeLists.txt`
- Modify: `src/pico_bridge/test/test_pico_calibration_menu.py`
- Create: `src/pico_bridge/test/test_pico_palm_orientation_script.py`

**Interfaces:**
- `./scripts/calibrate_pico_palm_orientation.sh left|right`.
- Uses existing service when present; otherwise starts one temporary side publisher with `setsid`, waits for service readiness, invokes the client, and reuses `pico_stop_process_group` for cleanup.

- [ ] Add failing script tests for side validation, service reuse, temporary publisher cleanup, successful result formatting, failed calibration, and Ctrl-C.
- [ ] Run focused tests and verify failures.
- [ ] Implement the ROS client and shell wrapper without duplicating calibration math.
- [ ] Run shell syntax checks and focused tests.

### Task 5: Launch, Documentation, and Full Verification

**Files:**
- Modify: `src/pico_bridge/launch/start_pico_palm_skeleton_filter.launch.py`
- Modify: `README.zh-CN.md`
- Modify: `README.md`
- Modify: `docs/PICO_PALM_SKELETON_FILTER.md`
- Modify: `src/pico_bridge/test/test_launch_integration.py`

**Interfaces:**
- Runtime launch passes the artifact path and calibration parameters to each publisher.
- Documentation gives the one-line left/right commands and service names.

- [ ] Add failing launch tests for side-specific service configuration and artifact paths.
- [ ] Update launch wiring and documentation.
- [ ] Run `bash -n` and `git diff --check`.
- [ ] Build `pico_bridge` with symlink install.
- [ ] Run the complete `pico_bridge` colcon test suite and require zero failures.
- [ ] Perform a read-only artifact check proving the active left geometry remains valid before hardware recapture.
