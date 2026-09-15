# PICO Upper-Arm Outward-Only Verification

Date: 2026-08-14

## Scope

The new `outward_only` redundancy mode keeps the existing PICO pose mapping,
Cartesian OTG, Cartesian servo, 13-variable velocity/acceleration QPs, joint
bounds, and reference integrators. It disables the arm-angle soft task and adds
one unilateral QP row that only prevents lateral upper-arm motion toward the
shoulder centerline.

Left-arm outward is world `+Y`; right-arm outward is world `-Y`. Elbow height
and forward/backward position are not constrained.

## Input and method

All modes replayed the same recorded bag:

```text
/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/
  recordings/pico_fast_motion_20260812_205428
duration: 29.888558786 s
```

Each mode used a newly started PICO bridge, a 36-second Viewer window, the
acceleration-level QP, and the same model/config. The mode-specific Viewer
argument was:

```text
--arm-angle-mode pico
--arm-angle-mode default_down
--arm-angle-mode outward_only
```

Outputs:

```text
/tmp/pico_pico_outward_comparison.csv
/tmp/pico_default_down_outward_comparison.csv
/tmp/pico_outward_only_outward_comparison.csv
/tmp/pico_outward_only_comparison.json
```

The report was generated with:

```bash
python3 scripts/compare_pico_arm_redundancy_modes.py \
  --pico /tmp/pico_pico_outward_comparison.csv \
  --default-down /tmp/pico_default_down_outward_comparison.csv \
  --outward-only /tmp/pico_outward_only_outward_comparison.csv \
  --output /tmp/pico_outward_only_comparison.json
```

The bridge accepted 2632-2634 packets per run. Because overwrite scheduling and
bridge resynchronization can select slightly different packets, the analyzer
uses the last live 200 Hz row for each `pico_sequence` and compares the 1988
sequences common to all three runs.

## Results

| Mode | Arm | Position P95 | Orientation P95 | Minimum outward distance | Cycles below -1 mm |
|---|---:|---:|---:|---:|---:|
| `pico` | left | 142.8 mm | 0.250 rad | -126.4 mm | 266 |
| `pico` | right | 139.0 mm | 0.259 rad | -70.1 mm | 22 |
| `default_down` | left | 147.4 mm | 0.279 rad | -131.3 mm | 1129 |
| `default_down` | right | 153.0 mm | 0.279 rad | -129.7 mm | 1137 |
| `outward_only` | left | 218.9 mm | 0.255 rad | +7.0 mm | 0 |
| `outward_only` | right | 152.3 mm | 0.329 rad | +3.3 mm | 0 |

For `outward_only`, the minimum QP barrier residual was within numerical noise
(`-1.42e-14` left, `-7.33e-15` right), with zero barrier violations and zero
feasibility clips.

Runtime counters:

| Mode | Control failures | Deadline misses |
|---|---:|---:|
| `pico` | 0 | 0 |
| `default_down` | 1 | 3 |
| `outward_only` | 3 | 0 |

The rejected cycles were reported as `infeasible_bounds`, before QP solution
validation. They occurred at fast-transition points and remain visible in the
report; they are not omitted from the run-level counters. No aligned accepted
PICO sequence used in the tracking table was rejected.

## Conclusion

The unilateral constraint works as intended: it eliminates lateral inward
upper-arm configurations without imposing a height or forward/backward target.
It is not free. On this aggressive replay it increases left-arm Cartesian P95
error by about 76 mm versus PICO arm-angle tracking and right-arm orientation
P95 by about 0.070 rad. Therefore `outward_only` is an explicit safety/posture
A/B mode, not a demonstrated tracking-performance replacement for `pico`.

## Verification

```text
focused upper-arm tests: passed
controller integration tests: passed
Python comparison test: passed
full serial CTest: 48/48 passed
```
