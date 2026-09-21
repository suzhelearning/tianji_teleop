# VR-fast scripted Viewer trajectories

Date: 2026-08-10

## Goal

Make Viewer keys `1` through `4` exercise motion representative of fast VR
teleoperation in both Cartesian OTG branches. The current shared trajectory
frequency is 0.10 Hz, so one cycle takes 10 seconds and does not load the
high-response controller meaningfully.

Also make an argument-free Viewer launch select the OTG profile implemented by
its branch. Today both branches silently default to
`config/qp_ik_hierarchical.yaml`, which makes `./build/tianji_qp_ik_viewer`
an ambiguous test command.

## Selected behavior

The branch-specific OTG configurations shall use:

```text
trajectories.frequency_hz: 1.0
```

All existing amplitudes remain unchanged:

```text
circle radius:               0.06 m
figure-eight width:          0.08 m
figure-eight height:         0.05 m
orientation amplitude:       0.35 rad
```

This changes the period from 10 seconds to 1 second. The analytical target
peaks are approximately:

```text
circle linear velocity:             0.377 m/s
figure-eight x velocity:            0.503 m/s
figure-eight z velocity:            0.628 m/s
orientation angular velocity:       2.199 rad/s
figure-eight z acceleration:        7.896 m/s^2
figure-eight z jerk:                99.22 m/s^3
```

These demands fit inside the previously selected MuJoCo high-response OTG
limits of 12 m/s^2 translation acceleration, 120 m/s^3 translation jerk,
40 rad/s^2 angular acceleration, and 400 rad/s^3 angular jerk. They therefore
stress controller tracking without intentionally asking the trajectory
generator to violate its configured envelope.

## Branch-specific default launch

The velocity branch shall default to:

```text
config/qp_ik_cartesian_otg_velocity.yaml
```

The acceleration branch shall default to:

```text
config/qp_ik_cartesian_otg_acceleration.yaml
```

An explicit `--config FILE` continues to override the default. No new command
line options or runtime speed-toggle state are introduced.

## Scope and unchanged behavior

- Keys `1`, `2`, `3`, and `4` keep their circle, figure-eight, orientation,
  and combined meanings.
- Manual marker dragging (`0` or `M`) is unchanged.
- Controller gains, Cartesian OTG limits, joint limits, watchdogs, trajectory
  amplitudes, and endpoint settling logic are unchanged.
- The base `feature/mujoco-cpp-qp-ik-v1` branch and its standard hierarchical
  profile remain unchanged.
- This remains a MuJoCo stress profile and is not approved for real hardware.

## Verification

Development follows test-first changes in each branch:

1. Add a configuration regression assertion that the branch-specific OTG
   profile has a 1.0 Hz scripted trajectory; observe it fail at 0.10 Hz.
2. Add a launch-options regression test for the branch-specific no-argument
   default while preserving explicit `--config` override behavior; observe it
   fail against the hierarchical default.
3. Apply the minimal configuration and default-selection changes.
4. Run the full CTest suite in each worktree.
5. Run each Viewer headlessly with no `--config` argument and require an
   accepted final state, zero control failures, and zero deadline misses.
6. Re-run a deterministic trajectory-speed check and require a period no
   longer than 2 seconds. The expected result is a 1-second period in both
   branch-specific profiles.
