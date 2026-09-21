# PICO MuJoCo Joint Reference/Feedback Live Plot Design

## Goal

Add the same in-process seven-joint diagnostic panel used by the PICO high-response Viewer to `feature/pico-mujoco-teleop-v1`, while completing the missing feedback overlay. The panel must show the active controller's planned joint position, velocity, acceleration, and jerk together with MuJoCo feedback and the corresponding limits.

This feature is diagnostic-only. It must not change PICO mapping, Cartesian OTG, arm-angle control, QP formulation, solver state, controller gains, limits, reference integration, or MuJoCo commands.

## Selected Approach

Port the plotting modules and their tests selectively instead of cherry-picking the full historical commit. The reference branch has the required queue, history, layout, and keyboard structure, but its current renderer stores feedback values without drawing them. The port will adapt the modules to the current branch and add the fourth `actual` curve explicitly.

Rejected alternatives:

- Cherry-picking `5e20667` directly would introduce avoidable conflicts because the Viewer and controller diagnostics have diverged.
- Extending CSV/PlotJuggler alone would not provide immediate in-process diagnosis and would not satisfy the requested Viewer behavior.

## Data Model and Semantics

Each 200 Hz control cycle produces one fixed-size bilateral `JointKinematicsSample` containing:

- sequence number and simulation time;
- left and right QP/reference `q`, `qdot`, `qddot`, and jerk;
- left and right MuJoCo feedback `q`, `qdot`, `qddot`, and jerk;
- effective lower and upper bounds for position, velocity, acceleration, and jerk;
- independent validity flags for reference and feedback acceleration/jerk;
- a reset marker.

Reference values have controller-specific meanings:

- Velocity-level QP:
  - `q_ref` is the accepted integrated controller reference.
  - `qdot_ref` is the accepted QP velocity/reference state.
  - `qddot_ref = (qdot_ref[k] - qdot_ref[k-1]) / 0.005`.
  - `jerk_ref = (qddot_ref[k] - qddot_ref[k-1]) / 0.005`.
- Acceleration-level QP:
  - `q_ref` and `qdot_ref` are the post-integration controller references.
  - `qddot_ref` is the accepted QP `qddot` output, not a feedback derivative.
  - `jerk_ref = (qddot_ref[k] - qddot_ref[k-1]) / 0.005`.

MuJoCo feedback uses `MujocoRobot::armPosition` and `armVelocity`. Feedback acceleration and jerk are fixed-step derivatives at the 0.005 s simulation/control period. Derivatives are calculated in the control thread before queueing, so Viewer frame rate and queue drops cannot change their values.

Position bounds come from the robot arm limits. Velocity plots use the effective QP velocity bounds when available, acceleration plots use the effective acceleration-QP bounds, and jerk plots use the configured hard jerk range. For a control level where a tighter dynamic bound is unavailable, the configured hard limit is shown. The plot labels must distinguish reference, actual, lower, and upper curves.

## Reset and Invalid-Data Handling

The differentiators reset on:

- controller/reference synchronization or nominal reset;
- pause/resume transition;
- velocity/acceleration control-level transition;
- rejected/frozen control output;
- invalid or non-finite source data;
- non-positive or unexpected control period.

After reset, acceleration and jerk remain invalid until enough consecutive samples exist. Invalid values are omitted from the relevant curves instead of drawing zeros or producing transition spikes. Reference validity is independent of feedback validity, so noisy or temporarily unavailable feedback cannot hide valid QP output.

## Threading and Performance

The control thread pushes complete samples through a dedicated bounded SPSC queue. `tryPush` is non-blocking; a full queue increments a display-drop counter and never blocks or changes controller behavior.

The Viewer thread drains the queue into a preallocated circular history. The history capacity is 2001 samples, and figures display the latest five seconds. Plotting and MuJoCo rendering occur only in the Viewer thread. No dynamic allocation or rendering API call is added to the 200 Hz control path after initialization.

## Viewer Interface

When the framebuffer is at least 1200 by 700 pixels, the panel occupies the right side and uses a 2-by-4 layout: seven joint figures plus one status cell. The MuJoCo scene remains on the left, and mouse interaction over the plot panel does not manipulate the scene.

Curves:

- cyan: QP/reference value;
- blue: MuJoCo actual value;
- red: lower and upper effective limits.

Controls:

- `F2`: show or hide the panel;
- `F3`: cycle `q -> dq -> ddq -> jerk`;
- `F4`: toggle left/right arm and lock that selection;
- `F5`: unlock and follow the currently selected arm.

The status cell reports selected arm and metric, control frequency, history occupancy, produced/dropped samples, and derivative validity.

## Integration Scope

New isolated modules own derivative/history handling and MuJoCo figure rendering. `run_qp_ik_viewer.cpp` only:

- creates the differentiators and queue;
- extracts current controller diagnostics and robot feedback;
- builds and queues samples;
- drains history and dispatches keyboard/rendering calls;
- exposes sample/drop/validity counters in `ViewerSnapshot` and headless output.

Small diagnostic fields may be added to controller diagnostics when necessary to expose already-computed effective bounds. They are read-only copies and must not feed back into the solver or reference integrator.

CSV telemetry remains supported. Adding the live panel must not rename or remove existing columns.

## Testing and Acceptance

Unit tests will cover:

- fixed-step reference and feedback derivatives;
- direct acceleration-QP output semantics;
- derivative warm-up, reset, invalid data, and source transitions;
- chronological circular-history behavior and independent validity flags;
- panel layout, hit testing, seven figures, four rendered curves, colors, labels, and bounds.

Headless integration tests will verify that velocity- and acceleration-level runs both produce finite bilateral samples, valid derivatives after warm-up, and bounded queue counters.

Acceptance requires:

- the project builds without changing controller configuration;
- all new unit and integration tests pass;
- both control levels show `q/qdot/qddot/jerk` reference and actual curves;
- reset/control-level transitions do not create artificial derivative spikes;
- disabling the panel leaves the control-loop behavior and telemetry unchanged;
- queue overflow can only drop visualization samples;
- existing PICO arm-angle source toggle remains operational.

## Non-Goals

- Filtering, smoothing, or limiting QP outputs;
- changing Cartesian targets or reducing the observed end-effector shake;
- replacing telemetry analysis tools;
- adding ROS 2 or PlotJuggler dependencies;
- changing collision, arm-angle, OTG, QP, solver, or MuJoCo control behavior.
