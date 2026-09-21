# Joint4 Human-Elbow Limit Design

## Goal

Prevent either arm's elbow joint from bending past the human-like straight-arm
position. `Joint4_L` and `Joint4_R` must have the same hard position range:

```text
lower = -2.5307 rad
upper = 0.0 rad
```

## Scope

Keep the existing lower limit and synchronize the new upper limit across every
checked-in representation of the robot model:

- the two source URDF variants under `marvin_m6_ccs/urdf/`;
- the local-path URDF under `models/`;
- the compiled runtime MJCF under `models/`.

The IK algorithms do not receive a separate elbow-specific override. Both the
hierarchical QP and null-space DLS paths continue to consume joint limits from
the loaded MuJoCo model, so the model remains the single runtime authority.

## Verification

Add a model-level regression test that loads the runtime MJCF, resolves both
Joint4 joints by name, and verifies their lower and upper limits. Run the full
CTest suite and the existing headless IK regression to detect any loss of
feasibility caused by the narrower range.

## Non-goals

- Changing Joint4 velocity, acceleration, or torque limits.
- Changing any other joint's position range.
- Adding a configurable controller-only Joint4 clamp.
- Regenerating meshes or changing kinematic transforms.
