# MuJoCo 6D Interactive Marker Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an RViz-style mouse-driven 6-DoF target marker to the MuJoCo Viewer so either arm can be commanded in position and orientation through the existing 1 kHz QP-IK loop.

**Architecture:** A Viewer-independent `InteractiveMarker6D` module owns analytic picking, constrained drag geometry, world/local frames, and pending-preview acknowledgement. A small MuJoCo adapter converts screen coordinates to world rays and appends decorative arrows, rings, and center geoms after `mjv_updateScene`; GLFW callbacks only translate input events into marker operations and existing bounded viewer commands.

**Tech Stack:** C++17, Eigen 3.4, MuJoCo C visualization API, GLFW, CMake, GoogleTest.

## Global Constraints

- Keep the controller and all MuJoCo control-state ownership at 1000 Hz unchanged.
- Do not add ROS, RViz, Python, or another rendering dependency.
- Manual marker input must publish the existing `ViewerCommandType::kSetManualTarget` command and must never mutate joint state directly.
- Queue-full, invalid-ray, and degenerate-constraint paths must remain non-blocking and must not publish non-finite poses.
- Existing solver, benchmark, safety, camera, trajectory, reset, pause, and headless behavior must remain compatible.
- Remote pushing remains paused until the user explicitly resumes it.

---

## File Structure

- Create `include/tianji_qp_ik/interactive_marker.hpp`: public marker handles, ray/camera/pointer data, drag state machine, visual geometry, and preview acknowledgement API.
- Create `src/interactive_marker.cpp`: finite-value validation, analytic picking, constrained translation/rotation, world/local axes, and preview selection.
- Create `tests/test_interactive_marker.cpp`: deterministic tests at the real interaction seam.
- Create `include/tianji_qp_ik/mujoco_marker_adapter.hpp`: MuJoCo camera-ray and decorative rendering declarations.
- Create `src/mujoco_marker_adapter.cpp`: `mjvScene` camera conversion and marker geom append logic.
- Create `tests/test_mujoco_marker_adapter.cpp`: camera-ray and bounded-scene adapter tests.
- Modify `apps/run_qp_ik_viewer.cpp`: replace the raw perturb target interaction with the marker state machine while preserving camera controls and command queues.
- Modify `CMakeLists.txt`: build the two modules and two tests.
- Modify `README.md`: document the visible handles, world/local frame toggle, and drag controls.
- Modify `docs/verification/qp_ik_v1_results.md`: record automated and GUI verification evidence.

---

### Task 1: Marker geometry, picking, and translation drag

**Files:**
- Create: `include/tianji_qp_ik/interactive_marker.hpp`
- Create: `src/interactive_marker.cpp`
- Create: `tests/test_interactive_marker.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `tianji_qp_ik::Pose` from `include/tianji_qp_ik/types.hpp` and Eigen vectors/matrices.
- Produces: `MarkerFrame`, `MarkerHandle`, `Ray3d`, `MarkerPointer`, `MarkerCamera`, `MarkerStyle`, `MarkerGeometry`, and `InteractiveMarker6D`.
- `InteractiveMarker6D::pick(const Pose&, const Ray3d&) const` returns the best matching handle or `kNone`.
- `beginDrag(MarkerHandle, const Pose&, const MarkerPointer&, const MarkerCamera&)` captures a drag constraint.
- `updateDrag(const MarkerPointer&)` returns an updated finite `Pose` or `std::nullopt`.

- [ ] **Step 1: Add failing picking and translation tests**

```cpp
TEST(InteractiveMarkerTest, PicksTranslationXArrow) {
  InteractiveMarker6D marker;
  const Pose pose = identityPose();
  const Ray3d ray{{0.12, -1.0, 0.0}, {0.0, 1.0, 0.0}};
  EXPECT_EQ(marker.pick(pose, ray), MarkerHandle::kTranslateX);
}

TEST(InteractiveMarkerTest, TranslationXPreservesOtherComponents) {
  InteractiveMarker6D marker;
  const Pose start = identityPose();
  const MarkerCamera camera = frontCamera();
  ASSERT_TRUE(marker.beginDrag(MarkerHandle::kTranslateX, start,
                               pointerAt({0.10, 0.0, 1.0}), camera));
  const std::optional<Pose> moved = marker.updateDrag(pointerAt({0.16, 0.0, 1.0}));
  ASSERT_TRUE(moved.has_value());
  EXPECT_NEAR(moved->position.x(), 0.06, 1e-9);
  EXPECT_NEAR(moved->position.y(), 0.0, 1e-12);
  EXPECT_NEAR(moved->position.z(), 0.0, 1e-12);
  EXPECT_TRUE(moved->rotation.isApprox(start.rotation, 1e-12));
}
```

- [ ] **Step 2: Register and run the focused test to verify it fails**

Run:

```bash
pixi run configure
pixi run build --target test_interactive_marker -j2
```

Expected: compilation fails because `interactive_marker.hpp` and its types do not exist.

- [ ] **Step 3: Define the marker API and implement minimal finite analytic picking/translation**

The header must expose these exact declarations:

```cpp
enum class MarkerFrame { kWorld, kLocal };
enum class MarkerHandle {
  kNone, kCenter, kTranslateX, kTranslateY, kTranslateZ,
  kRotateX, kRotateY, kRotateZ,
};

struct Ray3d {
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d direction{Eigen::Vector3d::UnitZ()};
};

struct MarkerPointer {
  Ray3d ray;
  Eigen::Vector2d ndc{Eigen::Vector2d::Zero()};
};

struct MarkerCamera {
  Eigen::Vector3d forward{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d right{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d up{Eigen::Vector3d::UnitY()};
  double world_per_ndc{1.0};
};

struct MarkerStyle {
  double scale{0.18};
  double center_radius_ratio{0.14};
  double arrow_start_ratio{0.22};
  double arrow_pick_radius_ratio{0.09};
  double ring_radius_ratio{0.72};
  double ring_pick_radius_ratio{0.09};
};

struct MarkerGeometry {
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 3> axes;
  MarkerStyle style;
};

class InteractiveMarker6D {
 public:
  explicit InteractiveMarker6D(MarkerStyle style = {});
  void setFrame(MarkerFrame frame) noexcept;
  MarkerFrame frame() const noexcept;
  MarkerHandle activeHandle() const noexcept;
  MarkerGeometry geometry(const Pose& pose) const;
  MarkerHandle pick(const Pose& pose, const Ray3d& ray) const;
  bool beginDrag(MarkerHandle handle, const Pose& pose,
                 const MarkerPointer& pointer, const MarkerCamera& camera);
  std::optional<Pose> updateDrag(const MarkerPointer& pointer) const;
  void endDrag() noexcept;
  void cancelDrag() noexcept;
};
```

Use ray/sphere distance for the center, ray/finite-segment distance for arrows, and normalized hit scores so the visually closest valid handle wins. Derive every drag update from the captured start pose. Reject non-finite/zero direction rays.

- [ ] **Step 4: Run the focused test and all existing math tests**

Run:

```bash
pixi run build --target test_interactive_marker test_so3 -j2
./build/test_interactive_marker
./build/test_so3
```

Expected: all tests pass.

- [ ] **Step 5: Commit the geometry seam**

```bash
git add CMakeLists.txt include/tianji_qp_ik/interactive_marker.hpp \
  src/interactive_marker.cpp tests/test_interactive_marker.cpp
git commit -m "feat: add testable 6D marker geometry"
```

---

### Task 2: Rotation, local frame, degeneracy, and preview acknowledgement

**Files:**
- Modify: `include/tianji_qp_ik/interactive_marker.hpp`
- Modify: `src/interactive_marker.cpp`
- Modify: `tests/test_interactive_marker.cpp`

**Interfaces:**
- Consumes: Task 1 marker types and methods.
- Produces: complete `kCenter` and `kRotateX/Y/Z` dragging plus `ManualTargetPreview`.
- `ManualTargetPreview::record(ArmSide, const Pose&, std::uint64_t)` stores the newest accepted command.
- `resolve(ArmSide, const Pose&, std::uint64_t)` returns the preview until the snapshot acknowledges its command ID.

- [ ] **Step 1: Add failing rotation, frame, invalid-input, and preview tests**

```cpp
TEST(InteractiveMarkerTest, RotateLocalZPreservesPosition) {
  InteractiveMarker6D marker;
  marker.setFrame(MarkerFrame::kLocal);
  Pose start = identityPose();
  start.rotation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  ASSERT_TRUE(marker.beginDrag(MarkerHandle::kRotateZ, start,
                               pointerAtRing(0.0), frontCamera()));
  const std::optional<Pose> moved = marker.updateDrag(pointerAtRing(0.4));
  ASSERT_TRUE(moved.has_value());
  EXPECT_TRUE(moved->position.isApprox(start.position, 1e-12));
  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(0.4, start.rotation.col(2)).toRotationMatrix() * start.rotation;
  EXPECT_TRUE(moved->rotation.isApprox(expected, 1e-9));
}

TEST(ManualTargetPreviewTest, KeepsNewestPoseUntilAcknowledged) {
  ManualTargetPreview preview;
  const Pose old_pose = identityPose();
  Pose requested = old_pose;
  requested.position.x() = 0.25;
  preview.record(ArmSide::kLeft, requested, 42U);
  const Pose pending = preview.resolve(ArmSide::kLeft, old_pose, 41U);
  EXPECT_TRUE(pending.position.isApprox(requested.position));
  EXPECT_TRUE(pending.rotation.isApprox(requested.rotation));
  const Pose acknowledged = preview.resolve(ArmSide::kLeft, old_pose, 42U);
  EXPECT_TRUE(acknowledged.position.isApprox(old_pose.position));
  EXPECT_TRUE(acknowledged.rotation.isApprox(old_pose.rotation));
}
```

Also assert center-plane translation, handle capture outside its hit area, world/local axis differences, NaN rejection, parallel-ray fallback, proper rotation determinants, and `cancelDrag()` behavior.

- [ ] **Step 2: Run the focused tests to verify the new cases fail**

Run: `pixi run build --target test_interactive_marker -j2 && ./build/test_interactive_marker`

Expected: rotation/center/preview assertions fail because the behavior is not implemented.

- [ ] **Step 3: Implement the remaining constraints and preview state**

Add:

```cpp
class ManualTargetPreview {
 public:
  void record(ArmSide side, const Pose& pose, std::uint64_t command_id);
  Pose resolve(ArmSide side, const Pose& snapshot_pose,
               std::uint64_t last_processed_command_id);
  void cancel(ArmSide side) noexcept;
  void cancelAll() noexcept;
};
```

Rotation uses ray/ring-plane intersection and signed `atan2`; its screen-tangent fallback uses captured NDC plus `MarkerCamera::world_per_ndc`. Center translation intersects the current ray with the captured camera-facing plane. Normalize rotations with `Eigen::Quaterniond(...).normalized().toRotationMatrix()` and reject all non-finite results before returning.

- [ ] **Step 4: Re-run the focused tests**

Run: `pixi run build --target test_interactive_marker -j2 && ./build/test_interactive_marker`

Expected: every marker and preview test passes.

- [ ] **Step 5: Commit complete interaction behavior**

```bash
git add include/tianji_qp_ik/interactive_marker.hpp src/interactive_marker.cpp \
  tests/test_interactive_marker.cpp
git commit -m "feat: constrain 6D marker dragging"
```

---

### Task 3: MuJoCo camera-ray and marker renderer adapter

**Files:**
- Create: `include/tianji_qp_ik/mujoco_marker_adapter.hpp`
- Create: `src/mujoco_marker_adapter.cpp`
- Create: `tests/test_mujoco_marker_adapter.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 `MarkerGeometry`, `MarkerHandle`, `MarkerPointer`, and `MarkerCamera`; MuJoCo `mjvScene`.
- Produces: `markerPointerFromScene(...)`, `markerCameraFromScene(...)`, and `appendInteractiveMarker(...)`.

- [ ] **Step 1: Add failing camera and render-capacity tests**

```cpp
TEST(MujocoMarkerAdapterTest, CenterCursorProducesForwardRay) {
  const mjvScene scene = testSceneCamera();
  const MarkerPointer pointer = markerPointerFromScene(scene, 0.5, 0.5, targetPose());
  EXPECT_TRUE(pointer.ray.direction.isApprox(expectedForward(scene), 1e-9));
}

TEST(MujocoMarkerAdapterTest, DoesNotExceedSceneCapacity) {
  mjvScene scene = sceneWithOneFreeGeom();
  appendInteractiveMarker(markerGeometry(), MarkerHandle::kNone,
                          MarkerHandle::kNone, &scene);
  EXPECT_LE(scene.ngeom, scene.maxgeom);
}
```

Add perspective corner-ray, orthographic origin-offset, finite camera basis, selected-handle highlighting, and full-render geom-count cases.

- [ ] **Step 2: Register and run the test to verify it fails**

Run: `pixi run build --target test_mujoco_marker_adapter -j2`

Expected: compilation fails because the adapter API does not exist.

- [ ] **Step 3: Implement camera conversion and decorative geoms**

Declare:

```cpp
MarkerCamera markerCameraFromScene(const mjvScene& scene,
                                   const Eigen::Vector3d& marker_origin);
MarkerPointer markerPointerFromScene(const mjvScene& scene, double relx,
                                     double rely,
                                     const Eigen::Vector3d& marker_origin);
void appendInteractiveMarker(const MarkerGeometry& geometry,
                             MarkerHandle hovered, MarkerHandle active,
                             mjvScene* scene);
```

Use `mjv_averageCamera` and `mjv_frustumHeight` for the ray. Append three `mjGEOM_ARROW` connectors, three rings of 48 `mjGEOM_LINE` segments, and one `mjGEOM_SPHERE`. Check `scene->ngeom < scene->maxgeom` before every append. Use X/Y/Z colors `(0.9,0.15,0.15)`, `(0.15,0.9,0.2)`, `(0.2,0.4,1.0)` and higher emission/alpha for hover or active state.

- [ ] **Step 4: Run adapter and marker tests**

Run:

```bash
pixi run build --target test_mujoco_marker_adapter test_interactive_marker -j2
./build/test_mujoco_marker_adapter
./build/test_interactive_marker
```

Expected: all tests pass without exceeding scene capacity.

- [ ] **Step 5: Commit the MuJoCo adapter**

```bash
git add CMakeLists.txt include/tianji_qp_ik/mujoco_marker_adapter.hpp \
  src/mujoco_marker_adapter.cpp tests/test_mujoco_marker_adapter.cpp
git commit -m "feat: render MuJoCo 6D marker handles"
```

---

### Task 4: Viewer input/state integration

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_interactive_marker.cpp`

**Interfaces:**
- Consumes: Tasks 1–3 marker modules and existing bounded `ViewerCommand` queue.
- Produces: visible selected-arm marker, hover/capture input, immediate preview, and `W` frame switching in the running Viewer.

- [ ] **Step 1: Add failing state-transition regression tests**

Extend the pure interaction tests to assert that switching target, switching away from manual mode, resetting, and focus loss call `cancelDrag()` and that an outside click returns `kNone` without changing the selected arm. These tests exercise helper functions extracted from the Viewer callback rather than GLFW globals.

- [ ] **Step 2: Run the regression tests to verify they fail**

Run: `pixi run build --target test_interactive_marker -j2 && ./build/test_interactive_marker`

Expected: new transition assertions fail until the Viewer-neutral transition helper exists.

- [ ] **Step 3: Integrate the marker into callbacks and rendering**

Update `ViewerApplication` with `InteractiveMarker6D marker`, `ManualTargetPreview preview`, `MarkerHandle hovered_handle`, and selected-arm state. Replace raw left-button `mjvPerturb` target motion with this sequence:

```cpp
const MarkerPointer pointer = currentMarkerPointer(application, x, y);
const Pose shown = shownTargetPose(application, application.selected_arm);
const MarkerHandle handle = application.marker.pick(shown, pointer.ray);
if (application.marker.beginDrag(handle, shown, pointer,
                                 currentMarkerCamera(application))) {
  application.hovered_handle = handle;
}
```

During active drag, publish only finite updates:

```cpp
if (const std::optional<Pose> target = application.marker.updateDrag(pointer)) {
  ViewerCommand command;
  command.type = ViewerCommandType::kSetManualTarget;
  command.side = application.selected_arm;
  command.target = *target;
  if (const std::optional<std::uint64_t> id = pushCommand(application, command)) {
    application.preview.record(command.side, command.target, *id);
  }
}
```

Make `pushCommand` return the accepted command ID. Use `preview.resolve(...)` when applying target mocap poses. Append marker geoms after `mjv_updateScene`. Add `W` frame toggle and show frame, hovered handle, and active handle in the overlay/help. Preserve right/middle/wheel camera behavior and allow target-sphere click selection through the existing `mjv_select` fallback.

- [ ] **Step 4: Build and run headless integration**

Run:

```bash
pixi run build -j2
./build/tianji_qp_ik_viewer --config config/qp_ik.yaml \
  --model models/marvin_m6_qp_test.xml --headless --duration 3
```

Expected: build succeeds; headless completes every scripted stage with no control, command, or snapshot failures.

- [ ] **Step 5: Commit Viewer integration**

```bash
git add apps/run_qp_ik_viewer.cpp tests/test_interactive_marker.cpp
git commit -m "feat: control QP IK with a 6D marker"
```

---

### Task 5: Documentation and complete verification

**Files:**
- Modify: `README.md`
- Modify: `docs/verification/qp_ik_v1_results.md`

**Interfaces:**
- Consumes: completed Viewer controls and test outputs.
- Produces: reproducible run instructions and recorded acceptance evidence.

- [ ] **Step 1: Update user controls and limitations**

Document `L/R`, translation arrows, rotation rings, center sphere, `W` world/local toggle, hover/capture semantics, camera controls, and that target motion still passes through target-step limiting and QP safety.

- [ ] **Step 2: Run formatting/static checks and all tests**

Run:

```bash
cmake --build build -j2
ctest --test-dir build --output-on-failure
git diff --check
```

Expected: all tests pass and `git diff --check` emits no output.

- [ ] **Step 3: Run GUI acceptance matrix**

Run `pixi run viewer`, then verify:

1. `L` and `R` each show one full gizmo.
2. X/Y/Z arrows constrain translation.
3. X/Y/Z rings constrain rotation.
4. Center sphere performs camera-plane translation.
5. `W` visibly changes world/local axes from a rotated pose.
6. Handle capture survives moving outside the handle.
7. `O` and `P` both follow dragged targets.
8. Right/middle/wheel camera controls, `N`, pause, and exit still work.
9. No visible target snap-back, control failures, command drops, or crashes occur.

- [ ] **Step 4: Record exact evidence and commit**

```bash
git add README.md docs/verification/qp_ik_v1_results.md
git commit -m "docs: verify MuJoCo 6D target control"
```

- [ ] **Step 5: Perform final release verification**

Run:

```bash
ctest --test-dir build --output-on-failure
./build/tianji_qp_ik_benchmark --config config/qp_ik.yaml \
  --model models/marvin_m6_qp_test.xml --samples 20000 \
  --seed 20260809 --output benchmark_results.csv
git status --short --branch
```

Expected: all tests pass, marker work introduces no 1 kHz control regression, and the worktree contains no uncommitted implementation files.
