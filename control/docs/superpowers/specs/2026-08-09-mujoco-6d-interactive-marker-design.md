# MuJoCo 6D Interactive Marker Design

## Goal

Replace the Viewer’s hard-to-select translucent mocap target with an RViz-style 6-DoF interactive marker. A user must be able to select either arm, see explicit position and orientation handles, and drag one handle to update only the intended Cartesian degree of freedom while the existing 1 kHz QP-IK controller follows the target.

This change is limited to the MuJoCo verification Viewer. It does not change the QP formulation, real-time control thread, robot model, solver selection, or real-robot interfaces.

## Confirmed problem

The current Viewer exposes only a 36 mm translucent target sphere. Selection depends on `mjv_select` hitting that small sphere, which is commonly coincident with and occluded by the end-effector. Pressing `L` or `R` selects a target, but the next left click on anything other than the sphere clears that selection. Position and orientation changes are hidden behind `Ctrl`/`Shift` modifiers and there are no visible axis handles or rotation rings. This does not provide an RViz Interactive Marker experience and can make manual IK appear non-functional.

## User interaction

- `L` and `R` select the left or right target. Clicking a target center also selects it.
- Only the selected target displays the full marker, avoiding overlapping dual-arm gizmos. The unselected target remains a clearly visible colored sphere and coordinate frame.
- The selected marker contains:
  - red, green, and blue translation arrows for X, Y, and Z;
  - red, green, and blue rotation rings for rotation about X, Y, and Z;
  - a center sphere for camera-plane translation.
- Hovering highlights one handle. Pressing and dragging captures that handle until release, even if the cursor moves away from it.
- Dragging an arrow changes only translation along its axis.
- Dragging a ring changes only orientation about its axis.
- Dragging the center sphere translates in the camera-facing plane.
- `W` toggles marker axes between world and target-local coordinates. The overlay always reports the active frame.
- Any manual drag automatically enters manual target mode. Existing `M`, trajectory-mode, pause, reset, solver-switch, camera, help, and exit controls remain available.
- Right drag, middle drag, and wheel retain their camera behavior. A left click outside a handle no longer silently destroys keyboard selection.

## Architecture

### `InteractiveMarker6D`

A new Viewer-independent module owns marker interaction state and geometry math. Its public inputs are a target `Pose`, marker frame, camera ray, pointer phase, and marker scale. Its outputs are the hovered/active handle and an optional updated target `Pose`.

The module does not depend on GLFW, the controller thread, queues, or MuJoCo model state. It uses Eigen for deterministic 3D geometry and exposes enough handle geometry for the render adapter. This boundary makes the user’s exact axis-constraint behavior testable without a GUI.

Handles are represented by a closed enum: none, center, translate X/Y/Z, and rotate X/Y/Z. Drag start stores the target pose and the geometric anchor needed for the selected constraint. Drag updates always derive from that captured start state, avoiding incremental drift.

### Picking and constrained motion

The Viewer converts the mouse pixel to a world-space camera ray using the active `mjvScene` camera frustum.

- Translation arrows use ray-to-axis closest points. Near-parallel cases fall back to projecting mouse displacement onto the screen-space axis; invalid or non-finite results reject the update.
- Rotation rings intersect the ray with the ring plane. The signed angle between the captured and current radial vectors is applied about the selected world-space axis.
- The center handle intersects rays with a camera-facing plane through the captured target position.
- Picking uses analytic ray tests against the same arrows, rings, and center sphere exposed for rendering. Handle tolerances scale with marker size so visual and clickable regions agree.

World-frame axes are the identity basis. Local-frame axes are the captured target rotation basis. Orientation updates are composed consistently in world coordinates and normalized before publication.

### MuJoCo render/input adapter

The existing Viewer remains responsible for GLFW callbacks and command publication. After `mjv_updateScene`, it appends decorative `mjvGeom` objects for the marker:

- `mjGEOM_ARROW` connectors render translation axes;
- short capsule/line segments render each rotation ring;
- a solid sphere renders the center handle;
- hover/active handles use brighter emission and increased alpha.

The adapter updates a local preview pose immediately during dragging and renders that preview until the control snapshot acknowledges the generated command ID. This prevents the next render snapshot from visually snapping the marker back while the command crosses the SPSC queue.

Each accepted drag update publishes the existing bounded `kSetManualTarget` command. Queue-full behavior remains non-blocking: the preview remains finite, the drop counter increments, and the next pointer update can retry. The 1 kHz control loop receives only a `Pose`; no rendering or input work enters the real-time thread.

## Safety and error handling

- All ray, pose, axis, angle, and quaternion values are checked for finiteness.
- Degenerate rays, camera/constraint parallelism without a usable fallback, and ring intersections near the center produce no command rather than a discontinuous target.
- Position and orientation step limits already enforced by `TargetManager` remain authoritative.
- Selecting or dragging a marker never mutates robot joint state directly.
- Reset, mode changes away from manual, target switching, and window focus loss cancel an active drag cleanly.

## Testing

Automated tests cover the real interaction seam:

- ray/sphere, ray/axis, and ray/ring handle picking;
- X/Y/Z translation preserves the two orthogonal position components and all orientation components;
- X/Y/Z rotation preserves position and rotates only about the selected axis;
- world/local frame behavior from a non-identity target orientation;
- camera-plane center translation;
- captured-handle behavior outside the visual hit region;
- near-parallel, zero-length, and NaN inputs reject updates;
- command/preview acknowledgement state prevents stale snapshots from overwriting a drag;
- existing headless Viewer command traversal and all QP-IK tests continue to pass.

Manual GUI verification covers both arms, all six axis handles, center translation, world/local toggle, camera controls, both solvers, reset during/after dragging, and visible IK following without target snapping.

## Acceptance criteria

1. From `pixi run viewer`, the selected arm shows visible three-axis arrows, three rotation rings, and a center sphere.
2. Every handle can be clicked and remains captured throughout its drag.
3. Each translation handle changes only its selected Cartesian axis in the active frame.
4. Each rotation handle changes only orientation about its selected axis in the active frame.
5. The robot follows the dragged target through the existing QP-IK controller with no direct joint manipulation.
6. Manual dragging does not block or add work to the 1 kHz control thread.
7. All automated tests pass and a full GUI interaction checklist passes for both arms and both solver backends.

## Non-goals

- Collision-aware target projection, self-collision avoidance, Cartesian obstacle constraints, force control, and real-robot command publication.
- Replacing MuJoCo’s renderer or adding ROS/RViz as a dependency.
- Changing QP weights, solver timing policy, or the dual-arm all-or-nothing safety behavior.
