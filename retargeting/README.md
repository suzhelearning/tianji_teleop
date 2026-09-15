# Owned hand retargeting

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-retargeting)](https://github.com/wuji-technology/wuji-retargeting/releases)

Hand pose retargeting system for Wuji Hand. High-precision retargeting based on adaptive analytical and key-vector optimization, with Wuji Glove as the recommended live input path. Apple Vision Pro, video files, Intel RealSense, ZED cameras, and MANUS-style external pipelines can also be used as input sources.

https://github.com/user-attachments/assets/72116289-7a33-4a6b-83ca-fb4d9aaece0d

**Get started below. For full documentation, see the [Retargeting Docs](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/) on Wuji Docs Center.**

## Repository Structure

```text
├── wuji_retargeting/                 // Core package: retargeter interface, optimizers, kinematics, coordinate transforms
│   ├── opt/                          // Optimizer implementations: adaptive analytical and key-vector
│   ├── viz/                          // Visualization tools for parameter tuning
│   └── wuji-description/             // Bundled URDF, MJCF, and meshes (ordinary source)
├── example/                          // Demonstration scripts for simulation and hardware control
│   ├── input_devices/                // Input device modules (Vision Pro, MediaPipe replay, video, RealSense, ZED, Wuji Glove)
│   ├── config/                       // YAML configuration files
│   ├── data/                         // Sample recording data
│   └── utils/                        // Helper utilities
└── README.md
```

## Quick Start

### Installation

Use the monorepo root installation and `.venv`; there is no separate hand
environment or editable upstream checkout. The populated model and simulation
assets are checked in here, together with their upstream licenses. No submodule
initialization or network clone is needed.

```bash
# From the monorepo root, after creating the root .venv:
.venv/bin/python -m pip install -e .
```

The retargeter uses NumPy, NLopt, Pinocchio (`pin`), PyYAML, and SciPy. Live Wuji
Glove input additionally uses `wuji-sdk`; the legacy hand backend uses
`wujihandpy`. MuJoCo is needed for simulation/tuning. Optional camera inputs
require MediaPipe/OpenCV, RealSense requires `pyrealsense2`, Vision Pro requires
`avp_stream`, and ZED requires its hardware SDK Python bindings. Dependency
versions and optional groups are owned by the root `pyproject.toml`.

The root build compiles `retargeting.wuji_retargeting._native` with pybind11
and a C++17 compiler. This is the required analytical geometry/gradient kernel,
not an optional Python fallback. It borrows the existing Pinocchio position and
Jacobian arrays, retains the same centimeter/Huber/epsilon conventions, and
avoids per-finger NumPy temporaries in every NLopt evaluation. Calibration,
joint limits, regularization, coupling, and filter semantics stay unchanged.

The owned Python API is `retargeting.wuji_retargeting.Retargeter`. YAML model
paths resolve relative to the YAML file, not the current working directory.

### Running

Wuji Glove is the recommended live input for development and demos.

```bash
# Run from the monorepo root:

# Recommended: Wuji Glove live input
.venv/bin/python -m retargeting.example.teleop_sim --input wuji_glove --hand right --glove-sn <YOUR_SN>
.venv/bin/python -m retargeting.example.teleop_real --input wuji_glove --hand right --glove-sn <YOUR_SN>

# Replay a recording (adaptive analytical optimizer)
.venv/bin/python -m retargeting.example.teleop_sim --play data/avp1.pkl --hand left

# Key-vector optimizer
.venv/bin/python -m retargeting.example.teleop_sim --play data/avp1.pkl --hand right --config config/vector/vector_avp.yaml
```

Other input sources — video, RealSense, ZED, and Vision Pro — use the same `teleop_*.py` entry with the matching flag. For full commands, Wuji Glove preparation, Wuji Hand 2, and the tuning tool, see the docs below.

For Manus, keep the existing paired `.mcal` calibrations in `manus/calibration/`.
After sourcing ROS 2 and building the native SDK collector with
`bash manus/build.sh`, use `.venv/bin/python -m manus.start_hand_teleop --list-users`
and `.venv/bin/python -m manus.start_hand_teleop --user <USER>`. This launches
both Python nodes from the root installation; it does not alter calibration
files. Hand offset calibration is
`.venv/bin/python -m retargeting.example.calibrate_offset --help`.

## Documentation

Full guides live on [Wuji Docs Center](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/):

- [Installation](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/installation/): Dependencies, input extras, Docker, and Apple Vision Pro setup
- [Quick Start](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/quick-start/): Simulation, real hardware, Wuji Glove input, and Wuji Hand 2
- [Parameter Tuning](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/tuning/): The interactive tuning tool and the recommended tuning order
- [API Reference](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/api/): Retargeter interface and config parameters
- [Appendix](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/appendix/): Algorithm principles, troubleshooting, and custom input device integration

## Citation

If you find this project useful, please consider citing:

```bibtex
@software{wuji2026retargeting,
  title={WujiHand Retargeting},
  author={Guanqi He and Wentao Zhang},
  year={2026},
  url={https://github.com/wuji-technology/wuji-retargeting},
  note={* Equal contribution}
}
```

## Acknowledgements

This project builds upon several excellent open-source projects:

- [MuJoCo](https://mujoco.org/) for physics simulation
- [dex-retargeting](https://github.com/dexsuite/dex-retargeting) for hand retargeting algorithms
- [DexPilot](https://arxiv.org/abs/1910.03135) for vision-based teleoperation insights
- [VisionProTeleop](https://github.com/Improbable-AI/VisionProTeleop) for Apple Vision Pro streaming

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
