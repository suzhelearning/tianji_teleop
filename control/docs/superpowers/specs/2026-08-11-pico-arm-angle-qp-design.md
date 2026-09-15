# PICO-Guided Arm-Angle QP Design

> Implementation status: the original weighted-secondary-task proposal below
> was superseded by the strict Cartesian-nullspace refinement in
> `2026-08-11-ee-priority-arm-angle-nullspace-design.md`. The descriptions in
> this document have been updated to match the implemented algorithm.

## Goal

Keep Cartesian end-effector position and orientation as a strict primary task
while using the seventh joint redundancy to make each Tianji elbow follow
the corresponding corrected PICO upper-limb shape. In standalone IK, with no
fresh PICO skeleton, use a deterministic elbow-down reference so the solver does
not choose an elbow-above-shoulder branch.

The feature applies to both the velocity-level hierarchical QP and the default
acceleration-level QP. Existing joint position, velocity, acceleration, braking,
reference-watchdog, and Cartesian slack behavior remains unchanged.

## Source Skeleton Contract

The bridge continues to consume `/pico/smpl_palm_corrected_ik`, paired exactly
with `/pico/smpl_palm_corrected/status`. It uses corrected positions, not raw
SMPL positions or elbow quaternions:

- left shoulder, elbow, wrist: indices `16`, `18`, `20`;
- right shoulder, elbow, wrist: indices `17`, `19`, `21`.

The palm IK already reconstructs each wrist from the calibrated palm TCP,
enforces calibrated upper-arm and forearm lengths, clamps maximum reach to
`ratio_max_stretch`, chooses the elbow on the two-sphere intersection circle
using the observed elbow branch, and retains the previous elbow when the branch
reference degenerates. The Tianji bridge reuses that corrected result and does
not solve a second human arm triangle.

For one side, define

```text
u_h = normalize(p_wrist - p_shoulder)

c_h = (p_elbow - p_shoulder)
      - u_h * dot(u_h, p_elbow - p_shoulder)

v_h = normalize(c_h)
```

`v_h` is the radial elbow direction in the plane perpendicular to the
shoulder-to-wrist axis. It represents arm swivel without depending on human arm
length. A projection norm below `1e-4 m` marks that side invalid for the current
frame.

The bridge maps this direction into the robot world basis with

```text
R_map = R_robot_midpoint * transpose(R_pico_shoulder)
v_robot_world_reference = R_map * v_h
```

`R_robot_midpoint` is identity in the current fixed robot midpoint frame. The
configured `pico_world_x_offset_m` affects only the two palm target translations;
it must not alter shoulder, elbow, wrist, or arm-angle reference geometry.

## Wire Protocol V2

Protocol V2 extends the UDP frame from 160 to 208 bytes:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | magic `TJVR` |
| 4 | 2 | version `2` |
| 6 | 2 | declared size `208` |
| 8 | 8 | sequence |
| 16 | 8 | tracking epoch |
| 24 | 8 | source timestamp ns |
| 32 | 8 | bridge monotonic send timestamp ns |
| 40 | 4 | flags |
| 44 | 56 | left target pose |
| 100 | 56 | right target pose |
| 156 | 24 | left arm radial direction XYZ |
| 180 | 24 | right arm radial direction XYZ |
| 204 | 4 | CRC32 over bytes `[0, 204)` |

The existing low four required flag bits remain set. Bit 4 means the left arm
direction is valid; bit 5 means the right arm direction is valid. An invalid
direction is encoded as zero with its validity bit clear.

The updated Viewer accepts both V1/160-byte pose-only packets and V2/208-byte
packets. V1 has no human arm reference and therefore selects the standalone
elbow-down behavior. A malformed V2 pose or CRC rejects the packet. A malformed
optional arm direction clears only that side's arm-reference validity so pose
teleoperation can continue with the fallback.

## Robot Arm-Angle Geometry

For each arm, the MuJoCo command-model sample at `q_ref` exposes:

- shoulder point `S`: `Link1` joint origin;
- elbow point `E`: `Link4` joint origin;
- wrist center `W`: `Link5` joint origin;
- translational Jacobians `J_S`, `J_E`, and `J_W`, obtained with
  `mj_jacBody`;
- existing TCP geometric Jacobian `J_tcp`.

Define

```text
u = normalize(W - S)

c = (E - S) - u * dot(u, E - S)
r = norm(c)
v = c / r
```

The incoming or default reference direction is projected into the plane normal
to `u` and normalized to `v_ref`. The signed arm-angle error is

```text
e_arm = atan2(
    dot(u, cross(v, v_ref)),
    dot(v, v_ref)
)
```

The arm-angle row is the exact first-order differential of this complete
geometry. For an infinitesimal joint displacement `dq`, let

```text
a = W - S
b = E - S
P_u = I3 - u * transpose(u)

da = (J_W - J_S) * dq
db = (J_E - J_S) * dq
du = P_u * da / norm(a)

dc = P_u * db
     - (du * transpose(u) + u * transpose(du)) * b

dv = (I3 - v * transpose(v)) * dc / norm(c)
```

For the selected world reference `h`, its normalized projection `p` and
differential are

```text
d = P_u * h
p = d / norm(d)

dd = -(du * transpose(u) + u * transpose(du)) * h
dp = (I3 - p * transpose(p)) * dd / norm(d)
```

With

```text
x = dot(v, p)
y = dot(u, cross(v, p))
e_arm = atan2(y, x)

de_arm = (x * dy - y * dx) / (x^2 + y^2)
J_arm * dq = -de_arm
```

`J_arm` therefore maps joint velocity to the current arm-angle rate while
`e_arm` is reference angle minus current angle. It includes shoulder, elbow,
wrist-axis, normalization, and projected-reference motion. The primary QP does
not contain an arm-angle Hessian or gradient term. After it succeeds, the
implementation projects `J_arm` into the exact numerical nullspace of the TCP
Jacobian and applies only a one-dimensional, joint-bound-clamped correction.
The correction is rejected unless its Cartesian residual remains within the
safety tolerance.

## Primary QP and Nullspace Refinement

For the velocity-level hierarchical QP, clamp

```text
arm_velocity_desired = clamp(
    kp_velocity * e_arm,
    -max_velocity,
    +max_velocity
)
```

The primary velocity QP first solves without an arm-angle objective. For its
accepted solution `qdot_primary`, compute a normalized Cartesian-nullspace
direction `n`, then apply

```text
qdot = qdot_primary + alpha * n

alpha = clamp(
    activation * (arm_velocity_desired - J_arm * qdot_primary)
      / (J_arm * n),
    alpha_lower_from_joint_bounds,
    alpha_upper_from_joint_bounds
)
```

to the existing objective.

For the acceleration-level QP, estimate

```text
arm_velocity = J_arm * qdot_model

arm_acceleration_desired = clamp(
    kp_acceleration * e_arm - kd_acceleration * arm_velocity,
    -max_acceleration,
    +max_acceleration
)
```

The acceleration controller applies the same bounded nullspace refinement to
the accepted primary `qddot` solution:

```text
qddot = qddot_primary + alpha * n
```

The acceleration task intentionally omits `Jdot_arm * qdot`; it is a bounded,
feedback-recomputed secondary refinement at every control sample. Cartesian
`Jdot_tcp * qdot` compensation remains unchanged. If no valid nullspace,
sensitivity, or alpha interval exists, the accepted primary solution is used
unchanged.

Default configuration is:

```yaml
arm_angle:
  enabled: true
  kp_velocity: 32.0
  max_velocity_rad_s: 24.0
  kp_acceleration: 80.0
  kd_acceleration: 18.0
  max_acceleration_rad_s2: 60.0
  minimum_radius_m: 0.015
  full_weight_radius_m: 0.050
  reference_rate_limit_rad_s: 8.0
```

All values must be finite. Gains are non-negative; limits and radii are
positive; `full_weight_radius_m` must exceed
`minimum_radius_m`.

## Reference Selection and Degeneracy

Each side owns a continuous unit-vector reference state:

1. A fresh, enabled V2 PICO frame with a valid side bit selects the human
   reference.
2. No configured PICO, PICO disabled, V1 input, stale input, or an invalid side
   selects world `-Z` (elbow down).
3. World-reference changes are rate-limited on the unit sphere. The resulting
   in-plane projected direction is independently reprojected from its previous
   state and rate-limited about the current shoulder-wrist axis. Disconnects,
   epoch resets, and crossings near an axis-parallel reference therefore cannot
   command an instantaneous elbow flip.
4. If projecting the selected direction into the current robot arm plane has
   norm below `1e-3`, reuse and reproject the previous valid projected
   direction. With no previous direction, use body-outward (`+Y` left, `-Y`
   right), projected into the arm plane.
5. Scale nullspace-refinement activation from zero at `minimum_radius_m` to one
   at `full_weight_radius_m`. Below the minimum radius, preserve the reference
   state but disable arm-angle motion for that sample.

The fallback exists inside the Viewer/controller path, not in the PICO bridge,
so standalone marker and scripted IK always receive deterministic elbow-down
behavior without requiring a ROS or UDP process.

## Diagnostics

Per arm, expose in snapshots/headless telemetry:

- reference source: `pico`, `default_down`, `previous`, or `degenerate`;
- arm-angle task active flag;
- signed arm-angle error radians;
- current arm-angle rate in `rad/s`;
- requested velocity in `rad/s` and requested acceleration in `rad/s^2` as
  separate fields;
- radial projection radius;
- elbow world Z and shoulder world Z.

The active flag means that the selected solver actually consumed the task; for
example, geometry can remain valid while the task is disabled or the DLS
backend is selected. These fields distinguish a weak/degenerate arm task from a
QP, bound, or target-tracking problem.

## Tests and Acceptance

PICO bridge tests cover corrected index selection, left/right direction mapping,
unit normalization, rigid-body transformation, degenerate projection flags,
the fact that the palm X offset does not affect arm direction, V2 packet layout,
CRC, and runtime transmission.

Viewer tests cover V1 compatibility, V2 decoding, malformed optional direction
fallback, reference rate limiting, signed-angle wraparound, left/right symmetry,
default-down selection, straight-arm weight fade, finite-difference validation
of all three MuJoCo point Jacobians and the complete arm-angle row, and
the absence of arm-angle Hessian-gradient contributions, and bounded
velocity/acceleration nullspace refinement.

End-to-end acceptance uses reachable fixed palm poses and requires:

- human-reference arm-angle error below `0.10 rad` after one second;
- standalone default arm-angle error below `0.10 rad` after one second;
- for the elbow-down test pose, elbow Z no more than `0.02 m` above shoulder Z;
- no regression of existing Cartesian pose tolerances, bounds, atomic dual-arm
  commit, stale-input hold, or solver-failure behavior;
- all repository tests passing;
- a 10-second PICO headless run with control-loop p99 below `5000 us`, zero
  control failures, and recorded deadline misses. Ordinary-Linux wake-up misses
  are reported separately from compute time; strict zero-miss hard real-time
  acceptance remains deferred until a PREEMPT_RT/real-time deployment is in
  scope.

## Verification Evidence (2026-08-11)

At 200 Hz, the fixed-pose one-second regressions produced:

| Control level | Left arm error | Right arm error | Left TCP position error | Right TCP position error |
|---|---:|---:|---:|---:|
| velocity | `0.0232 rad` | `0.0208 rad` | `0.222 mm` | `0.212 mm` |
| acceleration | `0.0673 rad` | `0.0673 rad` | `0.739 mm` | `0.739 mm` |

Both are below the `0.10 rad` and `1.0 mm` acceptance limits after exactly 200
cycles. Exact arm-angle rows also match central differences on both the
synthetic geometry and the actual left/right MuJoCo model.

Ten-second 72 Hz synthetic PICO runs produced:

| Control level | Cycle p99 | Deadline misses | Control failures | Settled max TCP position error | Settled max arm-angle error |
|---|---:|---:|---:|---:|---:|
| velocity | `564.8 us` | `0` | `0` | `0.124 mm` | `8.53e-6 rad` |
| acceleration | `1465.5 us` | `7` | `0` | `0.431 mm` | `0.00469 rad` |

“Settled” covers live samples after control time `2.0 s`. The acceleration
misses occurred despite a compute p99 far below the 5 ms period and are retained
as ordinary-host scheduling evidence, not hidden or reclassified.

## Repository Scope

Work stays in the existing branches and worktrees:

- PICO bridge: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1`,
  branch `feature/tianji-mujoco-teleop-v1`;
- Tianji Viewer/QP: `/home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1`,
  branch `feature/pico-mujoco-teleop-v1`.

No new worktree or branch is created. Existing unrelated/uncommitted PICO
calibration, position-offset, and end-effector-axis changes must be preserved.
