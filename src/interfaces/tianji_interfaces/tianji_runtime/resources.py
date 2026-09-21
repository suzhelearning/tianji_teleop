"""Single authority for locating workspace resources and native binaries.

Both the source checkout and an installed layout are supported:

* ``TIANJI_WORKSPACE`` (injected by Pixi) locates ``config/``, ``profiles/``,
  ``vendor/``, ``logs/`` and the default dataset;
* ament packages are found either in the workspace overlay
  (``install/<environment>/``) or through ``AMENT_PREFIX_PATH``.

Nothing here guesses through ``parents[n]`` or trusts the process cwd, so an
entry point keeps working from any directory.
"""

from __future__ import annotations

import os
from pathlib import Path

_MARKER = "pixi.toml"


class ResourceNotFound(RuntimeError):
    """A required resource or native binary is missing."""



def workspace() -> Path:
    """Checkout root explicitly supplied by Pixi or the operator."""
    declared = os.environ.get("TIANJI_WORKSPACE")
    if not declared:
        raise ResourceNotFound("set TIANJI_WORKSPACE to the absolute workspace path")
    root = Path(declared).expanduser()
    if not root.is_absolute() or not (root / _MARKER).is_file():
        raise ResourceNotFound(
            f"TIANJI_WORKSPACE={root} must be an absolute workspace path "
            f"containing {_MARKER}"
        )
    return root.resolve()


def environment_name() -> str:
    """Name of the active Pixi environment (used to find its install prefix)."""
    return os.environ.get("TIANJI_ENVIRONMENT", "default")


def install_prefix() -> Path:
    """colcon install prefix for the active environment."""
    return workspace() / "install" / environment_name()


def control_prefix() -> Path:
    """Install prefix of the native `control` environment.

    The native CMake projects install here rather than into the ament overlay:
    they have their own pinned dependency set and are built by
    ``bash/build_native.sh`` instead of colcon.
    """
    return workspace() / "install" / "control"


def ampp_prefixes() -> list[Path]:
    """Every prefix ament should search, nearest (workspace overlay) first."""
    prefixes = [install_prefix()]
    for entry in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if entry and Path(entry) not in prefixes:
            prefixes.append(Path(entry))
    return prefixes


def package_share(name: str, *relative: str) -> Path:
    """Locate ``share/<name>/<relative...>`` for an ament package.

    Raises :class:`ResourceNotFound` with an explicit build hint instead of
    returning a path that does not exist.
    """
    if not name or "/" in name or os.sep in name:
        raise ValueError(f"package_share expects a bare package name, got {name!r}")
    prefixes = ampp_prefixes()
    prefixes.insert(1, install_prefix() / name)
    for prefix in prefixes:
        candidate = prefix / "share" / name
        if not candidate.is_dir():
            continue
        target = candidate.joinpath(*relative) if relative else candidate
        if target.exists():
            return target
        raise ResourceNotFound(f"{name} is built but does not contain {Path(*relative)}")
    raise ResourceNotFound(
        f"ament package {name!r} is not built for environment {environment_name()!r}; "
        f"run 'pixi run build' (looked in {', '.join(str(p) for p in prefixes)})"
    )


def config_path(*relative: str) -> Path:
    """Locate a file under the checkout's ``config/`` directory."""
    target = workspace() / "config" / Path(*relative)
    if not target.exists():
        raise ResourceNotFound(f"missing workspace configuration: {target}")
    return target


def profiles_dir() -> Path:
    """Personnel calibration profiles directory."""
    return workspace() / "profiles"


def dataset_dir() -> Path:
    """Raw schema-v1 dataset root (``TIANJI_DATASET`` overrides)."""
    declared = os.environ.get("TIANJI_DATASET")
    if declared:
        return Path(declared).expanduser().resolve()
    return Path("/data/TianjiData/raw")


def compressed_dir() -> Path:
    """Compressed dataset root, a sibling of the raw root."""
    return dataset_dir().parent / "compressed"


def vendor_path(*relative: str) -> Path:
    """Locate a vendored SDK path under the checkout's ``vendor/``."""
    return workspace() / "vendor" / Path(*relative)


def native_executable(name: str) -> Path:
    """Locate a native executable built by the control environment.

    ``name`` must be a bare basename: this intentionally cannot address files
    through path separators, so a caller cannot escape the install tree.
    """
    if not name or name in (".", ".."):
        raise ValueError("native_executable expects a non-empty basename")
    if os.sep in name or (os.altsep and os.altsep in name):
        raise ValueError(f"native_executable expects a basename, got {name!r}")
    install = workspace() / "install" / "control"
    candidates = (
        install / "bin" / name,
        install / "lib" / name,
        install / "lib" / "mapped_palm" / name,
    )
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    raise ResourceNotFound(
        f"native executable {name!r} is not built; run 'pixi run build' "
        f"(looked in {install})"
    )


def controller_profile(name: str) -> Path:
    """Locate a native controller profile YAML by basename.

    The profiles are owned by ``tianji_controller`` and installed to
    ``share/tianji_controller/config/``. ``home.yaml`` — referenced *from inside*
    a profile as a relative path, which the C++ loader resolves against the
    profile's own directory — sits beside them, so an installed profile and its
    Home vector are always consistent with each other.

    ``name`` must be a bare basename: a profile never addresses a file through a
    path separator, matching the native loader's contract.
    """
    if not name or os.sep in name or (os.altsep and os.altsep in name):
        raise ValueError(f"controller_profile expects a basename, got {name!r}")
    installed = control_prefix() / "share" / "tianji_controller" / "config" / name
    if installed.is_file():
        return installed
    source = (workspace() / "src" / "tianji" / "tianji_controller" / "native"
              / "config" / name)
    if source.is_file():
        return source
    raise ResourceNotFound(
        f"controller profile {name!r} is not installed; run 'pixi run build-workspace' "
        f"(looked in {installed.parent})")


def controller_resource(profile: Path, value: str) -> Path:
    """Resolve a profile reference without replacing explicit/custom resources.

    Reviewed native profiles retain their source-relative description references
    byte-for-byte. When installed those references address the active description
    package instead, matching the native controllerResource loader.
    """
    reference = Path(value)
    if reference.is_absolute():
        return reference
    local = profile.resolve().parent / reference
    if local.exists():
        return local.resolve()
    description = "../../../tianji_description/"
    if value.startswith(description):
        return package_share("tianji_description", value[len(description):]).resolve()
    return local.resolve()
