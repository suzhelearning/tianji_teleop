# pico_ee_v131_velocity_qp full velocity control-path port

Reference checkout is supplied only for offline comparison through the
`V131_REFERENCE_ROOT` environment variable; no external checkout is needed
at runtime. Reference commit:
3cfa5108b12d21232ce13a1f0d84831ad525d294,
config/qp_ik_pico_ee_v131_velocity_qp_mujoco.yaml.

## Scope and configuration

Public backend remains `pico_ee_dexhand_qp`; algorithm diagnostic is
`pico_ee_v131_velocity_qp`. The former Dexhand core remains archived.
Only this backend selects the new profile; other factories retain their defaults.

The implemented path is:
timestamped target history → Cartesian OTG → adaptive reference servo →
single-stage qpOASES velocity QP → optional host joint trajectory processing.

`include/tianji_v131/velocity_profile.hpp` explicitly carries the original
velocity-profile constants. This is a compiled profile (rebuild after editing),
not a runtime reader of the reference application's YAML. The C++ VelocityQp
constructor also accepts a QpIkConfig. Legacy host `qp_*` settings are not
v131 tuning knobs. Host timestep, tolerances and maximum step remain effective.

- Cartesian OTG: original Ruckig<3> translation/orientation, position-mode
  translation, stationary hold/hysteresis, angular-axis change handling,
  stale-frame handling and settle criteria.
- Reference servo: near/far gain scheduling, reference-twist feedforward,
  3 m/s linear and 12 rad/s angular command norm caps.
- Target manager: timestamp deduplication, 5 Hz twist filtering, 50 ms timeout,
  prediction branches and original 0.05 m / 0.20 rad target increment guard.
  In the selected original translation-position mode, second-order target
  prediction is deliberately inactive, exactly as in the reference.
- QP: velocity/slack/task-scale 15 variables, zero posture, normalized velocity
  regularization, continuity, jerk trend, task scaling [0.75,1], singularity
  gain scheduling and finite-difference singularity escape.
- Bounds: 0.05 rad position margin, velocity, braking, acceleration/hard jerk
  and full-joint temporal envelope with relax_to_safety conflict handling.
- qpOASES: 50 working-set recalculations, 0.5 ms budget per solve attempt,
  warm start, cold retry, validated solution. On solver failure only, the
  original bounded velocity/acceleration-trend fallback is retained. Host
  diagnostics mark it degraded; persistent failure still trips the existing
  failure window. Invalid problems are not repaired by this fallback.

## Source fidelity

Numerical sources copied with namespace/include-prefix changes only:
MujocoRobot (mujoco_robot.cpp/.hpp),
cartesian_otg.cpp, cartesian_servo.cpp, target_manager.cpp,
dexhand_target_conditioner.cpp, hierarchical_qp.cpp,
hierarchical_qpoases_solver.cpp, pico_ee_v131_jacobian_svd.cpp,
pico_ee_v131_singularity.cpp, full_joint_temporal_envelope.cpp and so3.cpp.
The target-manager header drops an unused MuJoCo application include.
velocity_ik.cpp retains the original bound generators and helpers, omitting
unrelated algorithm factories/nullspace refinement.

velocity_qp.cpp is the host adaptation of controller/task/solver assembly;
boundedSolverFallback retains the numerical portion of
DualArmController::applyBoundedFallback. The separate adaptive-nullspace
algorithm and other reference application algorithms are not selected.

## Host integration boundaries

- Original MujocoRobot::armKinematicsAt(q_ref) supplies world-frame TCP and
  Jacobian. The original fast XML is bundled without mesh assets/geometries;
  all joint/body/site transforms, inertias and model limits remain unchanged.
  Pinocchio supplies the singularity-gradient samples, as in ordinary original
  v131, and fixed world/Base transforms at the host interface boundary.
- Current robot geometry, 36.5 mm TCP mapping compensation and Home stay intact.
  The fast model is now the backend's independent kinematic model, not a
  replacement for the viewer's robot or hand models.
- QP uses original fast-model position limits and revalidates the final
  candidate with margin, as in the reference controller. `--joint-limit-source
  urdf` configures producer/coordinator/executor extra protection, not QP bounds.
  The original asymmetric bounds fit within the current default URDF ranges.
- Original fast simulation speed is 4 rad/s (reference model joint user data).
  The dedicated producer/coordinator profile uses 0.02 rad per 5 ms.
  Effective speed remains min(4, host maximum step / dt) for embedding callers.
  Other backends retain their previous step contracts.
- Source timestamps and local receive time are passed to the original target
  manager; repeatedly solving one frame does not renew its freshness.
- Each arm owns q_ref/qdot/qddot. A validated QP result is integrated into its
  model immediately, without waiting for coordinator or simulator feedback.
  Ordinary feedback does NOT reconstruct velocities or overwrite this state.
  Coordinator commands only seed initialization after explicit reset. The
  model advances only in teleop. Non-teleop, stale input and host hard safety
  rejection reset the model. Soft OTG rejection retains model position and OTG
  state, clearing joint history like original clearHistory(side).
- Passthrough validates the model's single-tick step, not asynchronous output
  lag. The coordinator still enforces its transport step/position guards.
- Additional source conditioning, joint Ruckig and coordinator step clipping
  remain independently optional. Internal Cartesian OTG is distinct from
  optional joint Ruckig; QP temporal bounds are not an output smoother.
- PICO/Manus capture, skeleton normalization, retargeting, policy inference,
  recording, source authorization and return-to-Home logic are not replaced.
- The IK target overlay still displays the incoming target, not internal OTG
  reference state. Raw overlays remain read-only.

This reproduces the original model-state algorithm inside a distributed session.
Host source mapping, startup Home, transport timing and optional postprocessing
remain integration differences; do not equate this with identical live motion
under different inputs, initial states or additional processing settings.
No live-PICO tracking accuracy or physical-device qualification is claimed.
The fast profile rejects real-capability launches.

## Verification

Native `v131_qp_probe` checks original target/OTG/servo equivalence,
repeated-frame timeout, invalid timestamps, reset, adaptive near/far gain,
Cartesian cap, task scaling, velocity/acceleration/jerk bounds, fast motion,
independence from stale external feedback and bounded fallback. Factory probes cover both arms,
Home/translation/rotation/combined targets and unreachable-target recovery.
Existing archived Dexhand and pinocchio_qp probes also run in build_ik_sim.sh.

Isolated full direct-path session:
`/tmp/pico-sim-smoke-a3t5xh2i/result.json`, passed, 978 unchanged proposal→command
samples, both arms moving, raw recording and both overlays, disconnect return.
Additional conditioning + joint Ruckig + clipping:
`/tmp/pico-sim-smoke-vxce2uwu/result.json`, passed.
Legacy default pipeline smoke:
`/tmp/pico-sim-smoke-qtwjq670/result.json`, passed.

Full Python-suite environment gaps and final regression counts are recorded in
the full-port plan; these smoke tests do not establish all-product qualification.

### Model-state fidelity validation (supersedes earlier feedback-history tests)

`scripts/compare_v131_reference.py --reference-root ORIGINAL_CHECKOUT` compiles
an independent oracle against the ORIGINAL libtianji_qp_ik.a/DualArmController.
The oracle never links the port. The port trace intentionally holds external
feedback at Home while the internal model advances. Same original model limits,
initial joints and world targets are used in both processes.

Result: /tmp/v131-reference-compare-w9_l12vl/result.json, passed.
- Moving dual-arm trace: 600 frames, acceptance identical, max joint difference
  5.003653047452872e-11 rad.
- Static combined rotation/recovery: 600 frames, acceptance identical (original
  left OTG rejects ticks 23–28), max difference 7.912032140566794e-12 rad.
- Mesh-free vs original XML: both arms, 30 random configurations, world FK and
  Jacobian agree within 1e-12.

These are measured finite trace comparisons, not proof for all trajectories or
hardware. Original Pinocchio 3.9 and host Pinocchio 4 are separate builds.

Startup validates MuJoCo/Pinocchio world FK/Jacobian agreement at zero and an
alternating 0.031-rad probe, with the original 1e-5 tolerances. Mismatched TCP
models fail explicitly. Focused Python regression: 120 tests passed; legacy
pinocchio_qp session passed (/tmp/pico-sim-smoke-vv3osh02/result.json).

Updated direct session: /tmp/pico-sim-smoke-nxnp60qn/result.json, passed with
model_state_only=true and kinematics=mujoco_world. Extra processing session:
/tmp/pico-sim-smoke-rd2pb2__/result.json, passed. A separately regression-tested
Home interpolation endpoint fix prevents a one-ULP difference from blocking
the existing exact Home barrier; no return safety tolerance was widened.

Build: `pixi run -e ik-build bash scripts/build_ik_sim.sh`.
qpOASES lives in the isolated ik-build environment with local runtime RPATH.
No build/runtime dependency on the reference checkout.

### Review corrections

- Optional host joint Ruckig now follows model position targets (zero terminal
  velocity, retargeted every cycle), so speed saturation does not permanently
  discard displacement. Legacy backends retain velocity-mode behavior. This
  optional processing is separate from original internal Cartesian OTG.
- Solved-but-invalid QP results after cold retry are solver failures eligible
  for bounded fallback, matching original HierarchicalQpIk7/DualArmController.
  Core `accepted` remains false during fallback and `fallback_applied` is true;
  the host adapter marks the bounded command deliverable with degraded status.
- `TIANJI_ENABLE_V131` defaults OFF. The local simulation build opts in; the
  portable build explicitly opts out, removing MuJoCo/qpOASES dependencies from
  legacy executables. Full portable builds with v131 enabled are rejected.
  Deploy rejects v131-linked staging binaries before filesystem mutations.

Review-fix verification: 125 focused Python tests passed, including an actual
v131-disabled ELF and a disposable deployment-guard execution. v131 numerical
fault probes and position Ruckig saturation/catch-up probe passed. Original
1200-frame comparison remains identical within 5.004e-11 rad
(/tmp/v131-reference-compare-uo3t_kkk/result.json). Direct, optional-smoothed and
legacy pinocchio_qp session smoke passed. The pre-existing legacy Regrind moving
target probe still reports 0.0144063 m (required <0.005 m); it is not included in
the passing-test claim. Full portable SDK/runtime deployment was not exercised.
