"""Resource locations for the simulation-only PICO2 bare-hand route.

Resource ownership determines resolution:

* the V131 kinematics models, the simulation config and the native artifacts
  built by this package are *package* resources, so ``PACKAGE`` (this file's own
  directory) addresses them and they keep working from an installed copy;
* the shared MuJoCo display models belong to ``tianji_description``;
* the pinned Hand2 interpreter and vendored bridge belong to this package's
  source checkout, addressed through :func:`tianji_runtime.resources.workspace`.
  The startup launcher itself is installed with this Python package.

No path here is derived from the process cwd.
"""

from __future__ import annotations

from pathlib import Path

from tianji_runtime.resources import ResourceNotFound, package_share, workspace

PACKAGE = Path(__file__).resolve().parent

#: Where this package lives inside the source checkout. The pinned Hand2
#: runtime is created there by `pixi install --manifest-path
#: <checkout>/<SOURCE_PACKAGE>/tools/wuji_hand_native/pixi.toml`; it exists in
#: the checkout and never inside an installed copy of the package.
SOURCE_PACKAGE = Path("src/teleop_inputs/pico2_hands")

#: Shared display model, owned by tianji_description.
DISPLAY_MODEL = "marvin_m6_wuji2.xml"

#: Pinned Hand2 retargeting environment, relative to SOURCE_PACKAGE.
HAND_ENV = "tools/wuji_hand_native/.pixi/envs/default"


def display_model_path(name: str = DISPLAY_MODEL) -> Path:
    """The shared MuJoCo model both the simulator and the real viewer display.

    The ament share directory wins; the source checkout is the fallback, which
    keeps the simulation runnable before the workspace overlay has been built
    (the same ``ResourceNotFound`` covers "not built" and "built without the
    file", so there is no third branch).
    """
    try:
        return package_share("tianji_description", "models", name)
    except ResourceNotFound:
        candidate = workspace() / "src/tianji/tianji_description/models" / name
        if not candidate.is_file():
            raise
        return candidate


def hand_runtime_python() -> Path:
    """Interpreter of the pinned Hand2 retargeting environment.

    ``scripts/wuji_hand_native_launcher.py`` prepares the C++ worker's startup
    manifest under this interpreter because the workspace's own Python carries a
    different Pinocchio pin.
    """
    return workspace() / SOURCE_PACKAGE / HAND_ENV / "bin/python"


def hand_runtime_launcher() -> Path:
    """Start-up helper of the pinned Hand2 runtime.

    The installed helper runs under the isolated interpreter; its caller
    supplies the vendored bridge explicitly so it needs no checkout discovery
    or imports from the main environment.
    """
    return PACKAGE / "scripts/wuji_hand_native_launcher.py"


def hand_retargeting_root() -> Path:
    """Vendored official Hand2 bridge and model resources."""
    return workspace() / SOURCE_PACKAGE / "third_party/wuji_hand_retargeting"
