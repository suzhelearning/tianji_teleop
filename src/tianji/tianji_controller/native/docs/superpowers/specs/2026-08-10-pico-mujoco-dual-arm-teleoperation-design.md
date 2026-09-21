# PICO corrected-IK to Tianji MuJoCo dual-arm teleoperation design

Date: 2026-08-10

## Goal

Drive both Tianji MuJoCo TCP targets from the PICO palm-constrained
`/pico/smpl_palm_corrected_ik` skeleton while preserving the existing 200 Hz
Cartesian OTG and velocity/acceleration QP control path.

The operator's shoulder midpoint is rigidly attached to the midpoint of the
two Tianji arm bases.  The mapping is absolute and one-to-one: both target
position and orientation come from PICO, with no clutch, first-frame
orientation alignment, hand-to-TCP orientation offset, or pose scale.

This first version is for MuJoCo evaluation only.  It does not authorize real
robot output.

## Repositories and isolation

Implementation is isolated from the existing branches:

- Tianji worktree:
  `/home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1`
- Tianji branch: `feature/pico-mujoco-teleop-v1`, based on
  `feature/cartesian-otg-acceleration-qp-v1`
- PICO worktree:
  `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1`
- PICO branch: `feature/tianji-mujoco-teleop-v1`, based on
  `feat/pico-palm-tcp-v2`

The existing base, velocity-OTG, acceleration-OTG, and PICO feature branches
remain unchanged.  The acceleration-OTG viewer already supports the `V` and
`A` control-level switches, so one Tianji feature branch can compare both QP
control levels against the same PICO target stream.

## Authoritative PICO input

The bridge consumes:

- skeleton: `/pico/smpl_palm_corrected_ik`,
  `geometry_msgs/msg/PoseArray`;
- status: `/pico/smpl_palm_corrected/status`, `std_msgs/msg/String` JSON.

The accepted skeleton contract is:

- `frame_id == "pico"`;
- exactly 24 finite poses;
- non-zero, normalizable quaternions;
- `stream_valid == true`;
- `ik_frame_valid == true`;
- `left.corrected == true` and `right.corrected == true`;
- matching skeleton and status source timestamps;
- positive, current `tracking_epoch`.

The relevant SMPL indices are:

```text
SPINE2         = 6
LEFT_SHOULDER  = 16
RIGHT_SHOULDER = 17
LEFT_HAND      = 22
RIGHT_HAND     = 23
```

`LEFT_HAND` and `RIGHT_HAND` are the calibrated PICO palm TCP poses.  Their
positions and orientations are copied from the palm-constrained reconstruction.
The `_ik` variant changes only shoulder and elbow frame conventions; it does
not alter these palm poses.

The bridge pairs status and skeleton by source timestamp.  It does not send a
raw or partially corrected fallback side.

## Shoulder frame and absolute mapping

PICO and the Tianji MuJoCo world both use a right-handed frame with X forward,
Y left, and Z up.

For every accepted PICO frame, build the shoulder frame `T_G_S` in PICO world
`G` as follows:

```text
o_S = 0.5 * (p_left_shoulder + p_right_shoulder)

y_S = normalize(p_left_shoulder - p_right_shoulder)

z_raw = o_S - p_spine2
z_S = normalize(z_raw - y_S * dot(y_S, z_raw))

x_S = normalize(cross(y_S, z_S))
z_S = normalize(cross(x_S, y_S))

R_G_S = [x_S y_S z_S]
T_G_S = [R_G_S o_S; 0 1]
```

This follows the exoskeleton shoulder-belt construction in
`catkin_exoskeleton_ws`, but uses the midpoint of SMPL shoulder joints as the
requested attachment origin.  `SPINE2` supplies the stable torso-up direction.
Degenerate shoulder separation, spine direction, non-right-handed rotation, or
non-finite input rejects the complete bilateral frame.

The Tianji left and right first-joint origins are:

```text
p_joint1_left  = (0, +0.2115, 1.121) m
p_joint1_right = (0, -0.2115, 1.121) m
```

Therefore the fixed robot attachment frame `T_W_M` is:

```text
o_M = (0, 0, 1.121) m
R_W_M = Identity
```

For side `i`, map the PICO hand pose directly:

```text
T_W_target_i = T_W_M * inverse(T_G_S) * T_G_hand_i
```

Equivalently:

```text
p_W_target_i = o_M + R_W_M * R_G_S^T * (p_G_hand_i - o_S)

R_W_target_i = R_W_M * R_G_S^T * R_G_hand_i
```

The position scale is exactly `1.0`.  There is no per-side orientation offset,
initial-pose capture, robot-pose substitution, or automatic axis correction.
If the displayed TCP axes appear semantically rotated, that is reported as a
frame-contract observation rather than hidden by runtime calibration.

Because both position and orientation are expressed relative to the current
shoulder frame, walking, leaning, or turning the torso does not command common
motion of the two robot targets.  Arm and palm motion relative to the shoulder
belt remains.

## Transport architecture

Use a ROS 2 to UDP bridge rather than adding ROS 2 dependencies to Tianji.

```text
PICO corrected IK (~72 Hz)
    -> C++ ROS 2 bridge
    -> one atomic dual-arm UDP datagram
    -> Tianji UDP receiver thread
    -> latest-only SPSC exchange
    -> 200 Hz control thread
    -> TargetManager / Cartesian OTG
    -> velocity or acceleration QP
```

The PICO bridge is event-driven.  It sends one datagram for each accepted
matched skeleton/status pair and has no resampling timer.  ROS subscriptions
use sensor-data QoS with depth one so stale frames cannot build a queue.

The default destination is `127.0.0.1:15000`; address and port are parameters
for later cross-machine use.

The versioned packet contains:

```text
magic
protocol_version
packet_size
sequence
tracking_epoch
source_timestamp_ns
bridge_send_monotonic_ns
validity_flags
left position xyz
left quaternion xyzw
right position xyz
right quaternion xyzw
CRC32
```

Encoding is explicitly little-endian and independent of C++ struct padding.
Decoder tests use fixed golden bytes.  The receiver validates magic, version,
size, CRC, finite values, quaternion norms, sequence ordering, and epoch before
publishing a frame to the control thread.

## Tianji threading and atomicity

The existing viewer command queue remains a single-producer queue for GLFW UI
commands.  UDP must not become a second producer on that queue.

Instead, the UDP receiver owns a separate latest-only SPSC exchange carrying
one bilateral target frame.  Each 200 Hz control cycle reads at most the newest
frame.  Intermediate network frames may be superseded, but old commands can
never create a control backlog.

TargetManager gains an atomic bilateral timestamped update.  Both sides are
validated before either side changes.  Both target poses use the same sequence,
source timestamp, and receive time, so a partial left/right update is
impossible.

An epoch change clears target-velocity history and reinitializes the Cartesian
reference from current robot TCP states before accepting the new absolute
target.  The PICO target pose itself is not replaced or re-anchored.

## Multi-rate target processing

Mapped target velocity is estimated after shoulder attachment, not from raw
PICO world motion:

```text
dt_source = timestamp[k] - timestamp[k-1]

v_target_raw =
    (p_target[k] - p_target[k-1]) / dt_source

w_target_raw =
    LogSO3(R_target[k] * R_target[k-1]^T) / dt_source
```

Only a new accepted source frame updates the estimate.  Invalid or implausible
`dt_source` does not update velocity.  The existing 15 Hz timestamp-aware
filter remains the single velocity filter.

When Cartesian OTG is enabled, filtered target velocity is supplied to the
OTG target state.  It is not also injected as a second direct feedforward term.
The 200 Hz OTG produces continuous pose, velocity, and acceleration references
between the approximately 72 Hz PICO frames.

## Activation, hold, timeout, and jumps

The viewer adds an explicit PICO teleoperation enable state.  A command-line
option starts the UDP receiver; a viewer key toggles whether valid PICO frames
may update manual targets.  Disabling PICO teleoperation enters Hold and leaves
the receiver available for diagnostics.

On the first valid frame:

- initialize Cartesian OTG state from current robot TCP poses;
- switch to manual target mode;
- command the absolute mapped PICO targets;
- let OTG limits smooth any initial pose difference.

No first-frame pose or orientation alignment is performed.

The teleoperation profile uses a 50 ms target timeout.  At timeout:

- freeze the last accepted target pose;
- set target linear and angular velocity to zero;
- mark the stream stale;
- let OTG decelerate smoothly;
- do not continue prediction.

After initialization, a frame exceeding configured position or SO(3) jump
thresholds is rejected rather than fed to OTG.  Tracking-epoch changes use the
explicit reset path and are not treated as ordinary jumps.

## Configuration and observability

Add a dedicated Tianji PICO teleoperation YAML profile instead of silently
changing the existing acceleration profile.  It retains the high-response
Cartesian OTG and QP settings, with a 50 ms external-target timeout.

The Viewer overlay and telemetry report:

- PICO teleoperation enabled/live/stale state;
- accepted input frequency;
- latest frame age;
- current tracking epoch and sequence;
- superseded, malformed, CRC-failed, reordered, and epoch-reset counts;
- UDP receive-to-control latency;
- existing control-cycle p99 and deadline misses;
- left/right pose errors and QP diagnostics.

PICO-side diagnostics report accepted/rejected pairing counts, rejection
reason, output frequency, sequence, epoch, and send errors.

## Performance contract

This is a soft-real-time design on ordinary Linux, not a hard-real-time claim.
For same-host MuJoCo operation, acceptance targets are:

```text
corrected-IK input: nominal PICO rate, approximately 72 Hz
bridge output: one datagram per accepted source frame, no timer downsampling
Tianji control: 200 Hz
same-host bridge/receive latency p99: < 5 ms
control cycle p99: < 5 ms
normal-run control deadline misses: 0
left/right target source-time difference: 0
stale transition: <= 50 ms after the last accepted frame
```

Performance is verified with recorded or synthetic PICO frames and a headless
Viewer run before live MuJoCo evaluation.  Metrics are measured and reported;
they are not inferred from nominal frequencies.

## Testing

PICO tests cover:

- shoulder-frame geometry and degeneracy rejection;
- absolute one-to-one position and orientation mapping;
- body common-motion cancellation;
- exact use of `LEFT_HAND` and `RIGHT_HAND` orientations;
- exact skeleton/status timestamp pairing and corrected-side gating;
- protocol golden bytes, CRC, sequence, epoch, and malformed packet rejection;
- event-driven output with no duplicate frame sends.

Tianji tests cover:

- protocol decoding and quaternion validation;
- latest-only SPSC behavior under input bursts;
- atomic bilateral TargetManager updates;
- timestamp-aware mapped-target twist estimation;
- epoch reset and 50 ms stale transition;
- jump rejection and OTG smooth deceleration;
- coexistence with the independent UI command queue;
- a synthetic UDP-to-headless-Viewer integration test;
- unchanged velocity and acceleration QP regression suites.

## Out of scope

- Real Tianji hardware commands.
- Collision constraints or whole-body control.
- Position or orientation clutch mapping.
- First-frame or current-robot orientation alignment.
- Per-side hand-to-TCP orientation offsets.
- Position scaling other than `1.0`.
- Automatic correction when PICO and robot TCP local-axis semantics look
  different.
- Hard-real-time Linux or network guarantees.

