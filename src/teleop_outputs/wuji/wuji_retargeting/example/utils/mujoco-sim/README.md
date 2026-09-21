# mujoco-sim

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)  [![Release](https://img.shields.io/github/v/release/wuji-technology/mujoco-sim)](https://github.com/wuji-technology/mujoco-sim/releases)

Simulation demo for MuJoCo, shipped as package data of the retargeting workspace's `example` package. Loads the bundled MJCF models and plays pre-recorded trajectories in a loop; all models and meshes are ordinary source files.

**Get started with [Quick Start](#quick-start). For detailed documentation, please refer to [MuJoCo Simulation Example](https://docs.wuji.tech/docs/en/wuji-description/latest/related-repos/#41-mujoco-simulation-example) on Wuji Docs Center.**

https://github.com/user-attachments/assets/4b3d6d5c-420e-4e15-bbe7-68bcad9729f0

<video src="./assets/video.mp4" controls=""></video>

## Repository Structure

```text
├── assets/                    // Demo video files
│   └── video.mp4
├── data/                      // Pre-recorded trajectory data
│   └── wave.npy
├── wuji_hand_description/     // Bundled hand model (MJCF, meshes)
├── run_sim.py                 // Main simulation script
└── README.md
```

## Quick Start

### Prerequisites

- The Pixi environment that owns the workspace packages, with MuJoCo and NumPy installed.

### Installation

It is installed with the `wuji_retargeting` ament package as package data; no
separate environment, repository clone, or submodule initialization is required.
Dependencies live in the workspace `pixi.toml`.

### Running

```bash
cd "$(python -c 'import example, pathlib; print(pathlib.Path(example.__file__).parent / "utils/mujoco-sim")')"
python run_sim.py
```

The script loads the default right-hand model and plays the trajectory from `data/wave.npy` in a loop. To use the left hand, edit `side = "left"` in `run_sim.py`.

### Update Models

Model updates are ordinary reviewed source changes in this directory. Preserve
the model license and validate the MJCF mesh paths before changing assets.

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
