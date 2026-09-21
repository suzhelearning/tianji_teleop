# Tianji PICO tmux launcher design

## Goal

Provide one short command that starts and manages the complete PICO-side input
pipeline for Tianji Spark teleoperation:

```bash
./scripts/start_tianji_pico_teleop.sh
```

The launcher must not start the Tianji DLS/Spark MuJoCo viewer. The viewer
remains an independent process in the Tianji control repository.

## Scope

The launcher owns three PICO repository processes, each in a dedicated tmux
window:

1. `driver`: PICO ADB forwarding and the ROS 2 PICO bridge.
2. `m0`: calibrated palm, wrist, arm-geometry correction and upper-limb
   skeleton publication.
3. `bridge`: timestamp-matched corrected skeleton conversion and TJVR v4 UDP
   transmission to the Tianji receiver.

It does not build either repository, calibrate an operator, start the Tianji
viewer, or command real robot hardware.

## Command interface

The default command creates or attaches to the `pico_tianji_teleop` session:

```bash
./scripts/start_tianji_pico_teleop.sh
```

Management operations are:

```bash
./scripts/start_tianji_pico_teleop.sh --detach
./scripts/start_tianji_pico_teleop.sh --status
./scripts/start_tianji_pico_teleop.sh --stop
```

`--detach` starts the session without attaching. `--status` reports the
session and window state without changing it. `--stop` explicitly terminates
only the fixed launcher-owned tmux session.

## Startup behavior

Before creating a session, the launcher validates that `pixi`, `tmux`, and
`adb` are available, the PICO workspace has been built, and an authorized PICO
device is visible through ADB. Existing calibration validation remains owned
by `start_pico_m0.sh`, so there is one source of truth for artifact checks.

The launcher enters the repository Pixi environment automatically. Users do
not need to run `pixi shell`, export ROS variables, source the install space,
or type the ROS launch command. All windows inherit the standard local
defaults:

- `ROS_DOMAIN_ID=120`
- `EXO_REQUESTED_ROS_DOMAIN_ID=120`
- `ROS_LOCALHOST_ONLY=1`
- `ROS2CLI_DISABLE_DAEMON=1`
- destination `127.0.0.1:15000`
- position retargeting mode `robot_arm_segments`
- Tianji arm reach scale `0.95`

The three windows may start concurrently. The M0 readiness logic already
waits for its required upstream topics, while the event-driven UDP bridge can
remain idle until corrected frames arrive. tmux `remain-on-exit` preserves a
failed window so its error remains visible.

If the fixed session already exists, the default command attaches to it and
`--detach` returns successfully without creating duplicate ROS nodes.

## Documentation changes

`README.md` and `README.zh-CN.md` will use the one-command PICO-side launcher
as the primary Tianji quick start. Low-level multi-terminal commands remain
available only as an advanced troubleshooting reference.

Both READMEs will describe the current protocol and data path accurately:

- TJVR protocol version 4
- one 656-byte atomic bilateral UDP packet per accepted source frame
- corrected upper-limb positions and rotations
- left and right arm redundancy directions
- robot-segment position retargeting with a default reach scale of 0.95
- independent startup of the Spark qpOASES MuJoCo viewer

## Verification

Verification consists of:

1. Bash syntax checks for the new launcher and modified shell scripts.
2. Launcher help, status, and missing-session stop behavior checks.
3. A shell-level test using fake `tmux`, `adb`, and `pixi` executables to
   verify window creation, commands, duplicate-session handling, and stop
   isolation without requiring a headset.
4. Existing `pico_bridge` build and test suite.
5. Search checks ensuring both READMEs no longer describe the Tianji bridge as
   TJVR v1 or a 160-byte packet.

## Safety and lifecycle

The launcher never kills processes by name and never targets arbitrary tmux
sessions. Only `pico_tianji_teleop` is managed. Normal tmux detach leaves the
pipeline running. Explicit `--stop` terminates that session and its three
child process trees. The bridge remains simulation-oriented and does not send
commands to physical Tianji hardware.
