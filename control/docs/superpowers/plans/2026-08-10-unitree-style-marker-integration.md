# Unitree-Style RViz 6D Marker Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate Unitree-style screen-space gizmo selection into the existing MuJoCo 6D marker viewer while preserving the current analytic drag constraints, asynchronous QP-IK command path, camera controls, and safety behavior.

**Architecture:** Add a pure screen projection and nearest-handle picker to `mujoco_marker_adapter`. The viewer will convert GLFW window coordinates to framebuffer pixels, select a target with `mjv_select` before picking the selected marker, and use the screen picker only for mouse-down/hover; `InteractiveMarker6D` will continue to calculate captured drag constraints from MuJoCo rays. Valid poses continue through `ViewerCommand::kSetManualTarget` and `ManualTargetPreview`.

**Tech Stack:** C++17, Eigen, MuJoCo `mjvScene`/`mjv_cameraInModel`, GLFW, GoogleTest, CMake, existing SPSC command queue and QP-IK control thread.

## Global Constraints

- Work only in `/home/zj/current_robotics/TJ_arm/TJ_arm_control/.worktrees/qp-ik-v1` on branch `feature/mujoco-cpp-qp-ik-v1`; do not modify the main worktree.
- Preserve the three existing related uncommitted files and their camera-basis regression changes: `apps/run_qp_ik_viewer.cpp`, `src/mujoco_marker_adapter.cpp`, and `tests/test_mujoco_marker_adapter.cpp`.
- Use C++17 and the repository's existing Eigen/MuJoCo/GoogleTest conventions; do not add dependencies or modify the QP/controller/queue/safety architecture.
- Screen-space projection is authoritative for visible arrow/ring/center selection; analytic `InteractiveMarker6D::beginDrag` and `updateDrag` remain authoritative for the actual captured drag constraint.
- A blank click or drag must not enqueue `ViewerCommandType::kSetManualTarget`; queue pushes remain non-blocking and preview records only accepted commands.
- Every pose sent to the controller must remain finite and pass the existing `InteractiveMarker6D`/target safety path.
- Do not use destructive Git commands or reset existing user changes.

## File Map

- Modify `include/tianji_qp_ik/mujoco_marker_adapter.hpp`: expose screen-point and screen-pick result types plus pure MuJoCo-scene projection/picking functions.
- Modify `src/mujoco_marker_adapter.cpp`: implement projection, pixel segment distance, deterministic candidate ordering, arrow/ring/center projection, and invalid-input rejection while reusing render geometry constants.
- Modify `tests/test_mujoco_marker_adapter.cpp`: add red-green tests for screen projection, nearest arrow/ring/center selection, off-screen/degenerate input, and the existing camera-basis regression.
- Modify `apps/run_qp_ik_viewer.cpp`: add framebuffer cursor conversion, target-first mouse-down routing, screen-space hover/pick, captured drag routing, and focus/mode cancellation integration.
- Modify `README.md`: document arrow/ring/world-local controls and blank-area camera behavior; remove the obsolete Ctrl/Shift drag description.
- Do not modify `include/tianji_qp_ik/interactive_marker.hpp` or `src/interactive_marker.cpp` unless compilation proves a no-semantic-change helper is required; existing drag and preview tests already cover that module.
- Do not modify CMake because `test_mujoco_marker_adapter` already links the adapter and MuJoCo library.

---

### Task 1: Specify screen projection and picker behavior with failing tests

**Files:**
- Modify: `tests/test_mujoco_marker_adapter.cpp`
- Read only: `include/tianji_qp_ik/mujoco_marker_adapter.hpp`

**Interfaces:**
- Consumes the existing `MarkerGeometry`, `mjvScene`, `Eigen::Vector2d`, and test camera fixture.
- Produces tests for the following interfaces, which are intentionally absent before implementation:

```cpp
struct MarkerScreenPoint {
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
  double depth{0.0};
  bool visible{false};
};

struct MarkerScreenPick {
  MarkerHandle handle{MarkerHandle::kNone};
  double distance_pixels{std::numeric_limits<double>::infinity()};
  double depth{std::numeric_limits<double>::infinity()};
};

MarkerScreenPoint projectMarkerPoint(const mjvScene& scene,
                                     const Eigen::Vector3d& point,
                                     int viewport_width, int viewport_height);

MarkerScreenPick pickMarkerHandle(const MarkerGeometry& geometry,
                                  const mjvScene& scene,
                                  int viewport_width, int viewport_height,
                                  const Eigen::Vector2d& cursor_pixel);
```

- [x] **Step 1: Add screen-space test helpers and the first failing tests.**

Set `scene.scale = 1.0F` in `testCameraScene`, then add `<cmath>`, `<limits>`, `<Eigen/Geometry>`, and the following constants/helper immediately after `testCameraScene` in `tests/test_mujoco_marker_adapter.cpp`:

```cpp
constexpr int kPickerWidth = 1000;
constexpr int kPickerHeight = 1000;

Eigen::Vector2d projectedPixel(const mjvScene& scene, const Eigen::Vector3d& point) {
  const MarkerScreenPoint projected =
      projectMarkerPoint(scene, point, kPickerWidth, kPickerHeight);
  EXPECT_TRUE(projected.visible);
  return projected.pixel;
}
```

Add these tests below `CameraBasisAndScaleAreFinite`:

```cpp
TEST(MujocoMarkerAdapterTest, ProjectsCenterAndRejectsInvalidViewport) {
  const mjvScene scene = testCameraScene();

  const MarkerScreenPoint center =
      projectMarkerPoint(scene, Eigen::Vector3d::Zero(), kPickerWidth, kPickerHeight);
  ASSERT_TRUE(center.visible);
  EXPECT_TRUE(center.pixel.isApprox(Eigen::Vector2d(500.0, 500.0), 1e-9));
  EXPECT_GT(center.depth, 0.0);

  EXPECT_FALSE(projectMarkerPoint(scene, Eigen::Vector3d::Zero(), 0, kPickerHeight).visible);
  EXPECT_EQ(pickMarkerHandle(MarkerGeometry{}, scene, 0, kPickerHeight,
                             Eigen::Vector2d(500.0, 500.0)).handle,
            MarkerHandle::kNone);
}

TEST(MujocoMarkerAdapterTest, ScreenPickerSelectsNearestProjectedTranslationArrow) {
  const mjvScene scene = testCameraScene();
  InteractiveMarker6D marker;
  const MarkerGeometry geometry = marker.geometry(Pose{});
  const Eigen::Vector2d origin = projectedPixel(scene, geometry.origin);
  const Eigen::Vector2d x_end = projectedPixel(
      scene, geometry.origin + geometry.style.scale * geometry.axes[0]);
  const Eigen::Vector2d cursor = origin + 0.90 * (x_end - origin);

  const MarkerScreenPick pick =
      pickMarkerHandle(geometry, scene, kPickerWidth, kPickerHeight, cursor);

  EXPECT_EQ(pick.handle, MarkerHandle::kTranslateX);
  EXPECT_LT(pick.distance_pixels, 16.0);
}

TEST(MujocoMarkerAdapterTest, RejectsInvalidSceneScaleWithoutProjection) {
  mjvScene scene = testCameraScene();
  scene.scale = 0.0F;

  const MarkerScreenPoint projected =
      projectMarkerPoint(scene, Eigen::Vector3d::Zero(), kPickerWidth, kPickerHeight);

  EXPECT_FALSE(projected.visible);
}

TEST(MujocoMarkerAdapterTest, OrthographicProjectionIgnoresDepth) {
  const mjvScene scene = testCameraScene(true);
  const MarkerScreenPoint near_point = projectMarkerPoint(
      scene, Eigen::Vector3d(0.1, 0.0, 0.0), kPickerWidth, kPickerHeight);
  const MarkerScreenPoint far_point = projectMarkerPoint(
      scene, Eigen::Vector3d(0.1, 0.0, 1.0), kPickerWidth, kPickerHeight);

  ASSERT_TRUE(near_point.visible);
  ASSERT_TRUE(far_point.visible);
  EXPECT_NEAR(near_point.pixel.x(), far_point.pixel.x(), 1e-9);
  EXPECT_NEAR(near_point.pixel.y(), far_point.pixel.y(), 1e-9);
}

TEST(MujocoMarkerAdapterTest, ScreenPickerUsesLocalMarkerAxes) {
  const mjvScene scene = testCameraScene();
  InteractiveMarker6D marker;
  marker.setFrame(MarkerFrame::kLocal);
  Pose pose;
  pose.rotation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const MarkerGeometry geometry = marker.geometry(pose);
  const Eigen::Vector3d endpoint =
      geometry.origin + geometry.style.scale * geometry.axes[0];

  const MarkerScreenPick pick = pickMarkerHandle(
      geometry, scene, kPickerWidth, kPickerHeight, projectedPixel(scene, endpoint));

  EXPECT_EQ(pick.handle, MarkerHandle::kTranslateX);
}

TEST(MujocoMarkerAdapterTest, ScreenPickerSelectsProjectedRotationRing) {
  const mjvScene scene = testCameraScene();
  InteractiveMarker6D marker;
  const MarkerGeometry geometry = marker.geometry(Pose{});
  const Eigen::Vector3d normal = geometry.axes[1].normalized();
  const Eigen::Vector3d basis = normal.unitOrthogonal();
  const Eigen::Vector3d ring_point =
      geometry.origin + geometry.style.scale * geometry.style.ring_radius_ratio * basis;

  const MarkerScreenPick pick = pickMarkerHandle(
      geometry, scene, kPickerWidth, kPickerHeight, projectedPixel(scene, ring_point));

  EXPECT_EQ(pick.handle, MarkerHandle::kRotateY);
  EXPECT_LT(pick.distance_pixels, 1e-9);
}

TEST(MujocoMarkerAdapterTest, ScreenPickerSelectsCenterAndRejectsBlankCursor) {
  const mjvScene scene = testCameraScene();
  InteractiveMarker6D marker;
  const MarkerGeometry geometry = marker.geometry(Pose{});

  const MarkerScreenPick center = pickMarkerHandle(
      geometry, scene, kPickerWidth, kPickerHeight,
      projectedPixel(scene, geometry.origin));
  EXPECT_EQ(center.handle, MarkerHandle::kCenter);

  const MarkerScreenPick blank = pickMarkerHandle(
      geometry, scene, kPickerWidth, kPickerHeight, Eigen::Vector2d(20.0, 20.0));
  EXPECT_EQ(blank.handle, MarkerHandle::kNone);
  EXPECT_TRUE(std::isinf(blank.distance_pixels));
}
```

- [x] **Step 2: Run the targeted test to verify the new behavior is not already present.**

Run:

```bash
cmake --build build --target test_mujoco_marker_adapter -j2
```

Expected RED result: compilation stops at the test references to `MarkerScreenPoint`, `MarkerScreenPick`, `projectMarkerPoint`, and `pickMarkerHandle`, proving the tests are not accidentally exercising the existing analytic ray picker.

- [x] **Step 3: Commit only if the working-tree diff can be isolated; otherwise keep the red test in the implementation checkpoint.**

Because `tests/test_mujoco_marker_adapter.cpp` already contains the user's camera-basis and real-scene changes, do not stage the whole file merely to create a red-test commit. Preserve the complete diff for the implementation checkpoint and record the red command output before writing production code.

### Task 2: Implement the MuJoCo scene projection and deterministic screen picker

**Files:**
- Modify: `include/tianji_qp_ik/mujoco_marker_adapter.hpp`
- Modify: `src/mujoco_marker_adapter.cpp`
- Test: `tests/test_mujoco_marker_adapter.cpp`

**Interfaces:**
- Consumes the existing camera adapter and `MarkerGeometry` axes/style.
- Produces the exact `MarkerScreenPoint`, `MarkerScreenPick`, `projectMarkerPoint`, and `pickMarkerHandle` interfaces from Task 1.

- [x] **Step 1: Add the public result types and function declarations.**

Add `<limits>` to `include/tianji_qp_ik/mujoco_marker_adapter.hpp`, then add the result types and declarations before `markerCameraFromScene`:

```cpp
struct MarkerScreenPoint {
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
  double depth{0.0};
  bool visible{false};
};

struct MarkerScreenPick {
  MarkerHandle handle{MarkerHandle::kNone};
  double distance_pixels{std::numeric_limits<double>::infinity()};
  double depth{std::numeric_limits<double>::infinity()};
};

MarkerScreenPoint projectMarkerPoint(const mjvScene& scene,
                                     const Eigen::Vector3d& point,
                                     int viewport_width, int viewport_height);

MarkerScreenPick pickMarkerHandle(const MarkerGeometry& geometry,
                                  const mjvScene& scene,
                                  int viewport_width, int viewport_height,
                                  const Eigen::Vector2d& cursor_pixel);
```

- [x] **Step 2: Add projection and segment helpers in the adapter's anonymous namespace.**

Use the same MuJoCo camera model as the reference viewer, not a second hand-written projection matrix. The implementation must call `mjv_cameraInModel`, derive `right = forward.cross(up).normalized()`, use `mjv_frustumHeight`, reject non-positive depth and invalid viewport sizes, and return bottom-left-origin framebuffer pixels:

```cpp
double distanceToSegment(const Eigen::Vector2d& point,
                         const Eigen::Vector2d& begin,
                         const Eigen::Vector2d& end) {
  const Eigen::Vector2d segment = end - begin;
  const double squared_length = segment.squaredNorm();
  const double parameter = squared_length > kCameraEpsilon
      ? std::clamp((point - begin).dot(segment) / squared_length, 0.0, 1.0)
      : 0.0;
  return (point - (begin + parameter * segment)).norm();
}

MarkerScreenPoint projectMarkerPoint(const mjvScene& scene,
                                     const Eigen::Vector3d& point,
                                     int viewport_width, int viewport_height) {
  MarkerScreenPoint result;
  if (viewport_width <= 0 || viewport_height <= 0 || !point.allFinite()) {
    return result;
  }
  if (!std::isfinite(static_cast<double>(scene.scale)) ||
      static_cast<double>(scene.scale) <= kCameraEpsilon) {
    return result;
  }
  mjtNum head_array[3]{};
  mjtNum forward_array[3]{};
  mjtNum up_array[3]{};
  mjv_cameraInModel(head_array, forward_array, up_array, &scene);
  const Eigen::Vector3d head = vectorFromFloat3(head_array);
  const Eigen::Vector3d forward = vectorFromFloat3(forward_array).normalized();
  const Eigen::Vector3d up = vectorFromFloat3(up_array).normalized();
  const Eigen::Vector3d right = forward.cross(up).normalized();
  const Eigen::Vector3d relative = point - head;
  const double depth = relative.dot(forward);
  const double frustum_height = static_cast<double>(mjv_frustumHeight(&scene));
  if (!head.allFinite() || !forward.allFinite() || !up.allFinite() ||
      !right.allFinite() || !std::isfinite(depth) || depth <= kCameraEpsilon ||
      !std::isfinite(frustum_height) || frustum_height <= kCameraEpsilon) {
    return result;
  }
  const mjvGLCamera average = mjv_averageCamera(&scene.camera[0], &scene.camera[1]);
  const double scale = average.orthographic != 0
      ? static_cast<double>(viewport_height) / frustum_height
      : static_cast<double>(viewport_height) / (frustum_height * depth);
  result.pixel = Eigen::Vector2d(
      0.5 * static_cast<double>(viewport_width) + relative.dot(right) * scale,
      0.5 * static_cast<double>(viewport_height) + relative.dot(up) * scale);
  result.depth = depth;
  result.visible = result.pixel.allFinite();
  return result;
}
```

If MuJoCo's `mjtNum` is not accepted by `vectorFromFloat3`, add a local overload taking `const mjtNum[3]` with the same three scalar conversions; do not change the existing float camera helper or its corrected horizontal sign.

- [x] **Step 3: Implement nearest candidate ordering and projected geometry.**

Use `constexpr double kScreenPickPixels = 16.0`, `constexpr double kCenterPickPixels = 12.0`, and the existing `kRingSegments = 48`. Reuse `translationHandle`, `rotationHandle`, and `geometry.style` values already used by rendering:

```cpp
bool betterCandidate(double distance, double depth, MarkerHandle handle,
                     const MarkerScreenPick& best) {
  constexpr double kTieEpsilon = 1e-9;
  if (distance < best.distance_pixels - kTieEpsilon) {
    return true;
  }
  if (std::abs(distance - best.distance_pixels) <= kTieEpsilon &&
      depth < best.depth - kTieEpsilon) {
    return true;
  }
  return std::abs(distance - best.distance_pixels) <= kTieEpsilon &&
         std::abs(depth - best.depth) <= kTieEpsilon &&
         static_cast<int>(handle) < static_cast<int>(best.handle);
}
```

The picker algorithm is:

1. Reject invalid cursor, viewport, origin, scale, or non-finite/zero-length axes.
2. Project the marker center. If it is within `kCenterPickPixels`, return `kCenter` immediately so the visible center sphere remains usable even when an edge-on ring projection crosses the same screen pixel.
3. For each axis, project the visible arrow begin at `origin + scale * arrow_start_ratio * axis` and end at `origin + scale * axis`; compare cursor-to-segment distance against `kScreenPickPixels`.
4. For each axis, use `normal = axis.normalized()`, `basis_u = normal.unitOrthogonal()`, and `basis_v = normal.cross(basis_u).normalized()`. Project 48 ring segments at `ring_radius_ratio * scale`, compare consecutive visible segments against `kScreenPickPixels`, and record `kRotateX/Y/Z` according to the axis index.
5. Return the candidate with smallest pixel distance, then smallest depth, then enum order. Return the default `kNone` result when no candidate is within tolerance.

The center is retained as a screen-space candidate so the current captured view-plane center drag remains available. It is not an IK command until `InteractiveMarker6D::beginDrag(kCenter, ...)` and `updateDrag` produce a valid pose.

- [x] **Step 4: Run the adapter red-green cycle.**

Run:

```bash
cmake --build build --target test_mujoco_marker_adapter -j2
ctest --test-dir build -R '^test_mujoco_marker_adapter$' --output-on-failure
```

Expected result after implementation: the target builds and all `test_mujoco_marker_adapter` cases pass, including the existing 148-geometry and real-scene ray tests. If the camera-basis expectations fail, retain the existing `forward.cross(raw_up)` correction and fix only the new projection coordinate conversion.

### Task 3: Route viewer mouse events through target-first screen picking

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Test: `tests/test_mujoco_marker_adapter.cpp` and existing `tests/test_interactive_marker.cpp`

**Interfaces:**
- Consumes `projectMarkerPoint`/`pickMarkerHandle`, existing `markerPointerFromScene`, `markerCameraFromScene`, `mjv_select`, and `InteractiveMarker6D` drag methods.
- Produces viewer behavior where mouse-down resolves target first, screen picking starts capture, and subsequent movement never re-picks while a handle is active.

- [x] **Step 1: Add one viewport cursor conversion helper.**

Add this local type/function near `currentMarkerPointer`:

```cpp
struct ViewportCursor {
  int width{1};
  int height{1};
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
};

ViewportCursor viewportCursor(GLFWwindow* window, double cursor_x, double cursor_y) {
  int window_width = 1;
  int window_height = 1;
  int framebuffer_width = 1;
  int framebuffer_height = 1;
  glfwGetWindowSize(window, &window_width, &window_height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  window_width = std::max(window_width, 1);
  window_height = std::max(window_height, 1);
  framebuffer_width = std::max(framebuffer_width, 1);
  framebuffer_height = std::max(framebuffer_height, 1);
  const double scale_x = static_cast<double>(framebuffer_width) / window_width;
  const double scale_y = static_cast<double>(framebuffer_height) / window_height;
  return ViewportCursor{
      framebuffer_width,
      framebuffer_height,
      Eigen::Vector2d(cursor_x * scale_x,
                      (static_cast<double>(window_height) - cursor_y) * scale_y)};
}
```

Use framebuffer dimensions for both `mjv_select` and marker projection. This keeps GLFW top-left window coordinates, MuJoCo bottom-left viewport coordinates, and HiDPI framebuffer scaling in one conversion.

- [x] **Step 2: Update marker pointer and add a screen-pick helper.**

Change `currentMarkerPointer` to accept `const ViewportCursor& cursor` and compute:

```cpp
const double relx = std::clamp(cursor.pixel.x() / cursor.width, 0.0, 1.0);
const double rely = std::clamp(cursor.pixel.y() / cursor.height, 0.0, 1.0);
return markerPointerFromScene(application.scene,
                              static_cast<double>(cursor.width) / cursor.height,
                              relx, rely, target.position);
```

Add:

```cpp
MarkerHandle currentMarkerHandle(const ViewerApplication& application,
                                 const ViewportCursor& cursor) {
  const Pose target = mocapPose(application, application.selected_arm);
  const MarkerScreenPick pick = pickMarkerHandle(
      application.marker.geometry(target), application.scene,
      cursor.width, cursor.height, cursor.pixel);
  return pick.handle;
}
```

Change `beginMarkerDrag` to accept the cursor and call `currentMarkerHandle` rather than `InteractiveMarker6D::pick(target, pointer.ray)`. Keep `currentMarkerCamera` and the existing `beginDrag` call unchanged so the analytic constraint state is captured from the same pointer/camera.

- [x] **Step 3: Extract target selection and call it before handle picking.**

Move the current `mjv_select` block from `mouseButtonCallback` into:

```cpp
void selectTargetAtCursor(ViewerApplication& application,
                          const ViewportCursor& cursor) {
  mjtNum selected_point[3]{};
  int geom_id = -1;
  int flex_id = -1;
  int skin_id = -1;
  const int selected_body = mjv_select(
      application.robot.model(), application.robot.data(),
      &application.visual_options,
      static_cast<mjtNum>(cursor.width) / static_cast<mjtNum>(cursor.height),
      static_cast<mjtNum>(cursor.pixel.x()) / static_cast<mjtNum>(cursor.width),
      static_cast<mjtNum>(cursor.pixel.y()) / static_cast<mjtNum>(cursor.height),
      &application.scene, selected_point, &geom_id, &flex_id, &skin_id);
  if (selected_body == application.target_left_body) {
    selectTarget(application, ArmSide::kLeft);
  } else if (selected_body == application.target_right_body) {
    selectTarget(application, ArmSide::kRight);
  }
}
```

On left-button press, use this exact order:

```cpp
const ViewportCursor cursor = viewportCursor(window, application.previous_x,
                                             application.previous_y);
selectTargetAtCursor(application, cursor);
beginMarkerDrag(application, cursor);
if (application.marker.activeHandle() != MarkerHandle::kNone) {
  return;
}
```

If `mjv_select` does not hit a target, the current selected arm remains active and its projected handle can still be picked. If it hits another target, `selectTarget` cancels any stale drag and the picker uses the newly selected target, enabling a single click-drag to switch arm and edit.

- [x] **Step 4: Update hover, drag, and release paths without re-picking captured handles.**

Use `currentMarkerHandle` only in the no-button hover path. During an active left-button drag, call `currentMarkerPointer` and `application.marker.updateDrag(pointer)` directly; do not call the picker. Keep the accepted-command gate exactly as follows:

```cpp
const std::optional<Pose> target = application.marker.updateDrag(pointer);
if (target.has_value()) {
  ViewerCommand command;
  command.type = ViewerCommandType::kSetManualTarget;
  command.side = application.selected_arm;
  command.target = *target;
  const std::optional<std::uint64_t> command_id = pushCommand(application, command);
  if (command_id.has_value()) {
    application.preview.record(command.side, command.target, *command_id);
  }
}
```

Keep mouse release, `windowFocusCallback`, `selectTarget`, `sendMode`, `W`, `Space`, and `N` cancellation behavior. A failed ray update or full queue must not alter the last preview. A blank drag falls through to the existing MuJoCo camera mapping and sends no manual-target command.

- [x] **Step 5: Build and run focused interaction tests.**

Run:

```bash
cmake --build build --target tianji_qp_ik_viewer test_interactive_marker test_mujoco_marker_adapter -j2
ctest --test-dir build -R 'test_interactive_marker|test_mujoco_marker_adapter' --output-on-failure
```

Expected result: both focused tests pass, the viewer target builds, and the existing `ActiveHandleRemainsCapturedOutsidePickRegion` test continues to prove that the selected handle is not re-picked during drag.

### Task 4: Update user-facing controls and add integration checks

**Files:**
- Modify: `README.md`
- Modify if needed: `tests/test_mujoco_marker_adapter.cpp`
- Test: `build/tianji_qp_ik_viewer` headless path and full CTest suite

**Interfaces:**
- Consumes the final viewer behavior from Task 3.
- Produces documentation and repeatable verification evidence without changing controller semantics.

- [x] **Step 1: Replace obsolete viewer controls in README.**

Replace the existing mouse rows with the following table entries:

```markdown
| 左键拖动 XYZ 箭头 | 沿选定世界/局部轴平移当前目标 |
| 左键拖动 XYZ 旋转环 | 绕选定世界/局部轴旋转当前目标 |
| 左键拖动中心球 | 在当前相机视平面平移目标 |
| 左键拖动空白区域 | 只旋转相机，不发送 IK 目标命令 |
| `W` | 切换世界/局部 marker 坐标系 |
| 右键拖动 | 平移相机 |
| 中键拖动或滚轮 | 缩放相机 |
```

Keep the `L`/`R` target selection and mode rows. Explain that clicking either visible target first can switch the active arm before the same drag begins.

- [x] **Step 2: Run the complete automated suite.**

Run:

```bash
ctest --test-dir build --output-on-failure
```

Expected result: 16 or more tests pass with zero failures. The exact count may increase only if a new executable test target is added; no existing test may be removed or weakened.

- [x] **Step 3: Run the headless QP-IK acceptance command.**

Run:

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik.yaml \
  --model models/marvin_m6_qp_test.xml \
  --solver qpoases \
  --headless --duration 3
```

Expected output contains `headless_complete`, `backend=qpoases`, `control_failures=0`, `command_failures=0`, and `completed_stage=9`. Existing deadline and snapshot/telemetry statistics remain valid; marker changes must not alter headless command protocol.

- [ ] **Step 4: Perform manual GUI acceptance when a display is available.**

Run:

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik.yaml \
  --model models/marvin_m6_qp_test.xml \
  --solver qpoases
```

Verify all of the following: a visible arrow selects the expected axis; a visible ring rotates around the expected axis; the target can be switched by starting on the other target; a captured drag remains active after leaving the handle's projected area; blank drag moves only the camera; `W` changes the displayed frame; the horizontal screen direction is not mirrored; window focus loss does not leave a stuck drag.

The current automated environment only performed the GUI launch smoke test; the actual mouse drag checklist remains for manual confirmation.

- [x] **Step 5: Review the final diff and commit the implementation.**

Before staging, run:

```bash
git diff --check
git status --short
git diff --stat
```

Confirm the diff contains only the planned marker adapter, viewer, test, README, and plan files. Preserve the pre-existing camera-basis changes in the same related implementation diff; do not include build artifacts, telemetry CSVs, or unrelated files. Commit with:

```bash
git add include/tianji_qp_ik/mujoco_marker_adapter.hpp \
        src/mujoco_marker_adapter.cpp \
        tests/test_mujoco_marker_adapter.cpp \
        apps/run_qp_ik_viewer.cpp \
        README.md \
        docs/superpowers/plans/2026-08-10-unitree-style-marker-integration.md
git commit -m "feat: integrate screen-space marker picking"
```

## Self-Review Checklist

### Spec coverage

- Screen-space projected arrow/ring/center selection is covered by Task 1 and Task 2.
- MuJoCo camera basis, viewport orientation, and horizontal-sign regression are covered by Task 2 and the existing adapter tests.
- Target-first selection and one-click target switching are covered by Task 3.
- Captured handle lifetime, focus loss, mode changes, and target changes are covered by Task 3 and existing interactive-marker tests.
- Non-blocking command enqueue, accepted-command preview recording, queue-full handling, and no blank-area IK command are preserved by Task 3.
- README controls, full CTest, headless control loop, and GUI acceptance are covered by Task 4.
- QP, solver, controller, queue protocol, and SafetyGuard non-goals are enforced by the global constraints and file map.

### Placeholder and type checks

- Every new function name and return type is defined in Task 1 or Task 2 before viewer use in Task 3.
- The plan contains no unresolved implementation placeholder.
- The screen picker uses `MarkerGeometry` and the exact render constants rather than a second marker-size configuration.
- The existing `InteractiveMarker6D` analytic methods remain the only drag-state owner.

Plan complete and saved to `docs/superpowers/plans/2026-08-10-unitree-style-marker-integration.md`. Per the user's explicit request to start implementation, execute it in the current isolated worktree using `executing-plans`, with the tests and diff checks above as checkpoints.
