"""Shared deployment Home and controller initial-posture configuration."""
from pathlib import Path
import math

import yaml
from tianji_runtime.resources import controller_resource, package_share


def _joint_vector(value, name):
    if (not isinstance(value, (list, tuple)) or len(value) != 7
            or any(type(joint) not in (int, float) or not math.isfinite(joint) for joint in value)):
        raise ValueError(f"{name} must contain seven finite joint values")
    return tuple(float(joint) for joint in value)


def load_home(path=None):
    """Return the left/right Home vectors from one canonical-format YAML file."""
    if path is None:
        path = package_share("tianji_description", "config", "home.yaml")
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError("Home configuration must be a YAML mapping")
    return tuple(_joint_vector(data.get(f"{side}_home_rad"), f"{side}_home_rad")
                 for side in ("left", "right"))


def load_controller_posture(path):
    """Resolve Home or a complete explicit seed; None selects model midpoints.

    Explicit vectors take precedence without opening Home, including measured
    runtime configurations copied away from the source controller directory.
    """
    path = Path(path)
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or not isinstance(data.get("controller", {}), dict):
        raise ValueError("controller configuration must be a YAML mapping")
    controller = data.get("controller", {})
    has_home = "home_config" in controller
    enabled = controller.get("initial_posture_enabled", has_home)
    if not isinstance(enabled, bool):
        raise ValueError("initial_posture_enabled must be boolean")
    keys = ("initial_left_q_rad", "initial_right_q_rad")
    if any(key in controller for key in keys):
        if not all(key in controller for key in keys):
            raise ValueError("controller initial posture requires both left and right vectors")
        posture = tuple(_joint_vector(controller[key], key) for key in keys)
    elif has_home:
        home_path = controller["home_config"]
        if not isinstance(home_path, str) or not home_path.strip():
            raise ValueError("controller.home_config must be a nonempty path")
        posture = load_home(controller_resource(path, home_path))
    elif enabled:
        raise ValueError("enabled controller initial posture requires Home or both left and right vectors")
    else:
        posture = None
    return posture if enabled else None
