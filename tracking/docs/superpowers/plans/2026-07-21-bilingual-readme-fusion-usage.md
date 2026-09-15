# Bilingual README Fusion Usage Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide matching English and Simplified Chinese READMEs with current PICO, dual-IMU900 calibration, verification, and raw/fused MuJoCo overlay instructions.

**Architecture:** Keep `README.md` as the canonical English project overview and create `README.zh-CN.md` as its structurally equivalent Chinese translation. Both files use the same commands, links, section order, and technical values, with a language selector at the top.

**Tech Stack:** GitHub-flavored Markdown, ROS 2 CLI, Pixi workspace commands.

## Global Constraints

- `README.md` remains English and `README.zh-CN.md` uses natural Simplified Chinese.
- Executable commands, package names, topics, services, paths, and numeric configuration values remain byte-for-byte equivalent between languages; explanatory comments are translated.
- The recommended calibration is PICO A-button automatic zero-Z, one-second settling, and 60-frame sampling.
- Manual `/pico_foot_imu_fusion/calibrate` is documented as a fallback that does not reset IMU900 Z axes.
- The raw overlay uses `--show-raw --raw-topic /pico/smpl` and must be described as optional and non-blocking.
- Do not duplicate the complete troubleshooting and end-to-end test documents.

---

### Task 1: Update the English README

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: existing package list, build/run instructions, and current fusion documentation.
- Produces: canonical English headings, prose, commands, and links for the Chinese README.

- [ ] **Step 1: Add a language selector** immediately below the title:

```markdown
[English](README.md) | [简体中文](README.zh-CN.md)
```

- [ ] **Step 2: Update the MuJoCo section** to show the existing raw-only command and explain the optional same-coordinate comparison command:

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_fused \
  --show-raw \
  --raw-topic /pico/smpl \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0
```

State that fused geometry is blue/gray/orange, raw geometry is thinner transparent green, and missing/stale raw data does not block the fused skeleton.

- [ ] **Step 3: Replace the foot-fusion quick start** with ordered terminal commands for the PICO bridge and dual IMU launch, assigning `/dev/ttyUSB0` left and `/dev/ttyUSB1` right.

- [ ] **Step 4: Add verification commands** for `/imu/left_feet/ready`, `/imu/right_feet/ready`, both IMU rates, `/pico/smpl`, and `/pico/smpl_fused`.

- [ ] **Step 5: Document calibration behavior**: recommend standing still and pressing PICO controller A; describe sequential left/right zero-Z acknowledgements, one-second settling, cache discard, and 60 fresh neutral samples. Keep this fallback command and explicitly state that it does not issue hardware zero-Z:

```bash
ros2 service call /pico_foot_imu_fusion/calibrate std_srvs/srv/Trigger "{}"
```

- [ ] **Step 6: Link the detailed reference** using a repository-relative link to `docs/PICO_FOOT_IMU_FUSION.md`; do not present workstation-specific command logs as portable instructions.

- [ ] **Step 7: Validate and commit**:

```bash
git diff --check -- README.md
git add README.md
git commit -m "docs: update English PICO and IMU usage"
```

Expected: no whitespace errors and one English README commit.

### Task 2: Add the matching Chinese README

**Files:**
- Create: `README.zh-CN.md`

**Interfaces:**
- Consumes: final section order, commands, links, and facts from `README.md`.
- Produces: full Simplified Chinese project README with equivalent operational content.

- [ ] **Step 1: Translate all headings and prose naturally** while preserving the project title and this language selector:

```markdown
[English](README.md) | [简体中文](README.zh-CN.md)
```

- [ ] **Step 2: Copy every executable command unchanged** from the corresponding English section, including PICO bridge, Odin, recorder, IMU900, calibration, overlay, and mock-server commands; translate explanatory comments.

- [ ] **Step 3: Preserve technical identifiers** including topic names, service names, package names, paths, message types, array shapes, port assignments, 115200 baud, 110 Hz, one second, and 60 frames.

- [ ] **Step 4: Check structural and command parity** with a short Python read-only comparison that extracts fenced `bash` blocks from both files and requires equality:

```bash
python - <<'PY'
from pathlib import Path
import re

def bash_blocks(path):
    text = Path(path).read_text()
    return re.findall(r"```bash\n(.*?)```", text, flags=re.S)

english = bash_blocks("README.md")
chinese = bash_blocks("README.zh-CN.md")
def executable_lines(blocks):
    return [
        line for block in blocks for line in block.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]

assert executable_lines(english) == executable_lines(chinese)
print(f"matching executable lines: {len(executable_lines(english))}")
PY
```

- [ ] **Step 5: Validate links, whitespace, and required facts**:

```bash
rg -n "README.zh-CN.md|README.md|/pico/smpl_fused|--show-raw|calibrate|60|1 second|1 秒" README.md README.zh-CN.md
git diff --check -- README.md README.zh-CN.md
```

Expected: both language links and fusion facts are present with no whitespace errors.

- [ ] **Step 6: Commit and run repository verification**:

```bash
git add README.zh-CN.md
git commit -m "docs: add Simplified Chinese README"
pixi run test
pixi run bash -lc 'colcon test-result --test-result-base build --verbose'
```

Expected: the Chinese README is committed and `colcon test-result` reports zero errors and zero failures.

### Task 3: Final review and publish

**Files:**
- Review: `README.md`
- Review: `README.zh-CN.md`

**Interfaces:**
- Consumes: both completed README files.
- Produces: synchronized documentation on `personal/feature/pico-foot-imu-fusion`.

- [ ] **Step 1: Review the complete diff** for stale manual-calibration-first wording and mismatched commands.
- [ ] **Step 2: Confirm the branch contains only intended tracked changes** while leaving `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` and `pc_stream_records/` untracked.
- [ ] **Step 3: Push the current branch**:

```bash
git push personal feature/pico-foot-imu-fusion
```

Expected: local and `personal/feature/pico-foot-imu-fusion` resolve to the same commit.
