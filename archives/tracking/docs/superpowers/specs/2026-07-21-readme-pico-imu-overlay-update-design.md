# Bilingual README and PICO + IMU900 Usage Update Design

## Goal

Provide complete English and Simplified Chinese project READMEs so a new user
can start the existing PICO + dual-IMU900 pipeline, perform the recommended
automatic calibration, verify its topics, and compare raw and fused skeletons
in one MuJoCo coordinate frame.

## Scope

Update the existing English `README.md` and add a structurally equivalent
Simplified Chinese `README.zh-CN.md`. Each file will link to the other at the
top using an `English | 简体中文` language selector. The update will refine the
existing MuJoCo viewer and PICO + IMU900 foot-fusion sections rather than adding
a second full setup guide. Detailed troubleshooting remains in
`docs/PICO_FOOT_IMU_FUSION.md`, while the end-to-end command list remains in
`pico_full_test_commands.txt`.

## Content

Both READMEs will:

- use the same section order, executable commands, links, and technical facts;
- retain concise project-level wording, with natural English in `README.md` and
  natural Simplified Chinese in `README.zh-CN.md`;
- show PICO bridge and foot-fusion launch ordering with `/dev/ttyUSB0` assigned
  to the left foot and `/dev/ttyUSB1` assigned to the right foot;
- describe the recommended PICO A-button transaction: reset both IMU900 Z axes,
  wait one second, discard old samples, and collect 60 neutral samples;
- identify the manual `/pico_foot_imu_fusion/calibrate` service as a fallback
  that does not issue the hardware Z-axis reset;
- include ready, frequency, raw SMPL, and fused SMPL topic checks;
- document `--show-raw --raw-topic /pico/smpl` for same-coordinate overlay;
- explain fused colors (blue/gray/orange) and raw colors (transparent green),
  and state that a missing raw stream does not block fused rendering;
- link the detailed foot-fusion document without copying its full procedure or
  linking workstation-specific command logs as portable instructions.

The Chinese README will translate prose and headings but will not translate
executable commands, package names, topic names, service names, paths, or
configuration values. Explanatory shell comments are translated.

## Validation

- Check both resulting Markdown files for matching section structure, command
  blocks, links, and technical facts.
- Run `git diff --check`.
- Verify every documented executable, launch file, service, and topic name against
  the current repository.
