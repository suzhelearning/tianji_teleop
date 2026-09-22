# Owned hand retargeting

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-retargeting)](https://github.com/wuji-technology/wuji-retargeting/releases)

Hand pose retargeting system for Wuji Hand. High-precision retargeting based on adaptive analytical and key-vector optimization, with Wuji Glove as the recommended live input path. Apple Vision Pro, video files, Intel RealSense, ZED cameras, and MANUS-style external pipelines can also be used as input sources.

https://github.com/user-attachments/assets/72116289-7a33-4a6b-83ca-fb4d9aaece0d

**Get started below. For full documentation, see the [Retargeting Docs](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/) on Wuji Docs Center.**

## Repository Structure

```text
├── package.xml / setup.py / setup.cfg   // ament_python package definition; colcon builds `_native` from here
├── wuji_retargeting/                 // Core package: retargeter interface, optimizers, kinematics, coordinate transforms
│   ├── native.cpp                    // C++17 analytical geometry/gradient kernel (`wuji_retargeting._native`)
│   ├── opt/                          // Optimizer implementations: adaptive analytical and key-vector
│   ├── viz/                          // Visualization tools for parameter tuning
│   └── wuji-description/             // Bundled URDF, MJCF, and meshes (installed package data)
├── example/                          // Runnable commands for simulation and hardware control (top-level `example` package)
│   ├── input_devices/                // Input device modules (Vision Pro, MediaPipe replay, video, RealSense, ZED, Wuji Glove)
│   ├── config/                       // YAML configuration files (installed package data)
│   ├── data/                         // Sample recording data (installed package data)
│   └── utils/                        // Helper utilities
└── README.md
```

## Quick Start

### Installation

This tree is the `wuji_retargeting` ament package of the Tianji workspace
(`src/teleop_outputs/wuji/wuji_retargeting`). The Pixi-managed Jazzy environment and
`colcon build --base-paths src` build it, including the pybind11 extension;
there is no separate hand environment or editable upstream checkout. The
populated model and simulation assets are checked in here, together with their
upstream licenses. No submodule initialization or network clone is needed.

```bash
# From the workspace root, in the Pixi environment that owns this package:
pixi run install
```

The retargeter uses NumPy, NLopt, Pinocchio (`pin`), PyYAML, and SciPy. Live Wuji
Glove input additionally uses `wuji-sdk`; the legacy hand backend uses
`wujihandpy`. MuJoCo is needed for simulation/tuning. Optional camera inputs
require MediaPipe/OpenCV, RealSense requires `pyrealsense2`, Vision Pro requires
`avp_stream`, and ZED requires its hardware SDK Python bindings. Dependency
versions and optional groups are owned by the workspace `pixi.toml`.

The package build compiles `wuji_retargeting._native` from
`wuji_retargeting/native.cpp` with pybind11 and a C++17 compiler. This is the
required analytical geometry/gradient kernel, not an optional Python fallback.
It borrows the existing Pinocchio position and Jacobian arrays, retains the same
centimeter/Huber/epsilon conventions, and avoids per-finger NumPy temporaries in
every NLopt evaluation. Calibration, joint limits, regularization, coupling, and
filter semantics stay unchanged.

The owned Python API is `wuji_retargeting.Retargeter`; the runnable commands are
the `example` package. YAML model paths resolve relative to the YAML file, not
the current working directory.

### Running

Wuji Glove is the recommended live input for development and demos.

```bash
# Run in the Pixi environment with the workspace overlay sourced (install/<env>/local_setup.bash):

# Recommended: Wuji Glove live input
python -m example.teleop_sim --input wuji_glove --hand right --glove-sn <YOUR_SN>
python -m example.teleop_real --input wuji_glove --hand right --glove-sn <YOUR_SN>

# Replay a recording (adaptive analytical optimizer)
python -m example.teleop_sim --play data/avp1.pkl --hand left

# Key-vector optimizer
python -m example.teleop_sim --play data/avp1.pkl --hand right --config config/vector/vector_avp.yaml
```

Other input sources — video, RealSense, ZED, and Vision Pro — use the same `teleop_*.py` entry with the matching flag. For full commands, Wuji Glove preparation, Wuji Hand 2, and the tuning tool, see the docs below.

The workspace Manus route now uses ROS acquisition, semantic landmarks and the
official Wuji SDK Hand2 `RetargetSession`, not this legacy Pinocchio pipeline.
Keep wearer calibrations in `profiles/<USER>/manus/`; build with
`bash bash/build_manus.sh`, then run `bash bash/run_manus.sh --user <USER>`.
It publishes named 20-joint ROS targets only and does not execute hardware.
This library and its other offline tools remain available independently.

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
