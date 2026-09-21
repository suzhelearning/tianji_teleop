# SMPL Raw and Fused Overlay Design

## Goal

Allow the MuJoCo SMPL visualizer to display the fused skeleton and the original
PICO skeleton at the same world coordinates so foot-IMU corrections can be
compared directly.

## Command-line interface

The existing `--topic` remains the primary/fused PoseArray topic. Two optional
arguments are added:

- `--show-raw`: enable the second skeleton; disabled by default;
- `--raw-topic`: original skeleton topic, default `/pico/smpl`.

Without `--show-raw`, rendering and ROS subscriptions remain equivalent to the
current single-skeleton behavior.

## Rendering

The primary/fused skeleton keeps the existing blue joints, grey bones, and
orange oriented foot boxes. The original PICO skeleton uses separate MuJoCo
bodies and free joints, rendered as thinner semi-transparent green joints,
bones, and oriented foot boxes. Both streams use the same scale, viewer yaw,
and world coordinates; no comparison offset is applied.

Most non-foot geometry is expected to overlap. The smaller translucent raw
geometry preserves visibility of the primary skeleton while making differences
at foot indices 10 and 11 directly visible.

## Data and stale-state handling

The node creates a second best-effort PoseArray subscription only when
`--show-raw` is set. Primary and raw streams keep independent positions,
orientations, receipt times, and warnings under the existing lock.

A missing raw frame does not block primary rendering. Each stream independently
uses `--timeout`; stale primary geometry and stale raw geometry change to
separate muted colors. Camera initialization continues to use the primary pelvis
and therefore does not depend on the raw topic.

Invalid raw PoseArrays are rejected without modifying the last valid raw frame.
Node shutdown and the render loop remain shared.

## Verification

Unit tests cover the new parser defaults and switch, XML generation with and
without raw bodies, unique MuJoCo names for both skeletons, and distinct raw
geometry styling. The package test suite and full workspace build must pass.
Manual validation runs the overlay command and confirms that the two skeletons
overlap except where `/pico/smpl_fused` replaces the foot poses.
