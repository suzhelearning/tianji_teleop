"""MuJoCo dynamics for the TJRC joint stream, not a model-state viewer.

The source MJCF is never changed on disk. Its inertias, gravity, mesh collision
geometry and joint limits are retained. Gains below are simulation tuning, NOT
hardware calibration. There is no gravity compensation: loaded joints can sag
and contacts can prevent a target from being reached.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from control.home_config import load_controller_posture
from control.model_assets import add_object_mesh

from real_robot.protocol import (
    ARMS_READY,
    LEFT_HAND_READY,
    RIGHT_HAND_READY,
    CommandFrame,
)


HAND_STEMS = (
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
    "index_finger_mcp_flex", "index_finger_mcp_abd", "index_finger_pip", "index_finger_dip",
    "middle_finger_mcp_flex", "middle_finger_mcp_abd", "middle_finger_pip", "middle_finger_dip",
    "ring_finger_mcp_flex", "ring_finger_mcp_abd", "ring_finger_pip", "ring_finger_dip",
    "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
)
JOINT_NAMES = tuple(f"Joint{i}_{side}" for side in ("L", "R") for i in range(1, 8)) + tuple(
    f"{side}_{stem}" for side in ("l", "r") for stem in HAND_STEMS
)
# Nm/rad and Nm*s/rad. Native actuator damping is integrated implicitly, which
# matters for the small, original finger inertias (no invented rotor inertia).
ARM_KP = (250.0, 250.0, 180.0, 180.0, 60.0, 40.0, 25.0)
ARM_KV = (25.0, 25.0, 18.0, 18.0, 6.0, 4.0, 2.0)
HAND_KP = (0.8, 0.25, 0.4, 0.2) * 5
HAND_KV = (0.025, 0.015, 0.012, 0.008) * 5
# Accommodate float32 setpoint conversion at an XML boundary, then clamp only
# this representational overrun. This is not a motion/safety margin.
TARGET_TOLERANCE_RAD = 1e-7


def _simulation_model(model_path: Path, object_mesh: Path | None = None) -> mujoco.MjModel:
    root = ET.parse(model_path).getroot()
    if root.tag != "mujoco" or root.find("include") is not None:
        raise ValueError("simulation requires a self-contained MJCF model")
    root.set("model", "Tianji SIM dynamics - simulation gains, not hardware calibration")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    # from_xml_string has no source-directory context. Resolve all file assets
    # before compilation, without changing the original files or process cwd.
    assetdir = compiler.get("assetdir", "")
    for asset in root.findall("./asset/*"):
        filename = asset.get("file")
        if filename is not None:
            directory = compiler.get("meshdir" if asset.tag == "mesh" else "texturedir", assetdir)
            asset.set("file", str((model_path.parent / directory / filename).resolve()))
    for attribute in ("meshdir", "texturedir", "assetdir"):
        compiler.attrib.pop(attribute, None)
    compiler.set("fusestatic", "false")
    compiler.set("autolimits", "true")
    add_object_mesh(root, object_mesh)

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.001")
    option.set("integrator", "implicitfast")
    option.set("solver", "Newton")
    option.set("iterations", "100")
    option.set("tolerance", "1e-8")
    flag = option.find("flag")
    if flag is None:
        flag = ET.SubElement(option, "flag")
    for feature in ("gravity", "contact", "limit", "actuation", "filterparent"):
        flag.set(feature, "enable")

    world = root.find("worldbody")
    if world is None:
        raise ValueError("model has no worldbody")
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    body_names = {body.get("name") for body in world.iter("body")}
    for side in ("L", "R"):
        # The shoulder housing overlaps its directly mounted Link1 by 5.2 mm.
        # Give that fixed housing its own body so its exclusion does not also
        # disable Link1 collisions with every other world geom (stand/base).
        for geom in list(world.findall("geom")):
            if geom.get("mesh") == f"Base_{side}":
                housing = ET.SubElement(world, "body", name=f"sim_Base_{side}")
                world.remove(geom)
                housing.append(geom)
                ET.SubElement(contact, "exclude", body1=f"sim_Base_{side}", body2=f"Link1_{side}")
        # Link5/6/7 form a co-located two-axis wrist. Convex collision hulls of
        # Link5 and Link7 overlap by ~61 mm even at wrist zero. The intermediate
        # Link6 makes them grandparent/child, outside MuJoCo's parent filter.
        pairs = [(f"Link5_{side}", f"Link7_{side}")]
        # A finger's flexion and abduction bodies form one compound knuckle.
        # Palm/proximal_abd hulls overlap at that adjacent joint assembly, too.
        pairs.extend((f"Link7_{side}", f"{side.lower()}_{finger}_proximal_abd")
                     for finger in ("thumb", "index_finger", "middle_finger", "ring_finger", "pinky"))
        for first, second in pairs:
            if first in body_names and second in body_names:
                ET.SubElement(contact, "exclude", body1=first, body2=second)
    # All other contacts stay enabled, including fingers with each other,
    # non-adjacent arm links, hands against the stand, and opposite arms.

    existing_actuators = root.find("actuator")
    if existing_actuators is not None:
        root.remove(existing_actuators)
    actuators = ET.SubElement(root, "actuator")
    joints = {joint.get("name"): joint for joint in world.iter("joint")}
    gains_p = ARM_KP * 2 + HAND_KP * 2
    gains_v = ARM_KV * 2 + HAND_KV * 2
    for name, kp, kv in zip(JOINT_NAMES, gains_p, gains_v):
        if name not in joints:
            raise ValueError(f"model is missing TJRC joint {name}")
        joint = joints[name]
        limits = joint.get("range")
        forces = joint.get("actuatorfrcrange")
        if limits is None or forces is None:
            raise ValueError(f"{name} requires XML joint and actuator force limits")
        joint.set("limited", "true")
        joint.set("actuatorfrclimited", "true")
        ET.SubElement(actuators, "position", name=f"sim_{name}", joint=name,
                      kp=str(kp), kv=str(kv), ctrllimited="true", ctrlrange=limits,
                      forcelimited="true", forcerange=forces)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    if model.nq != 54 or model.nv != 54 or model.nu != 54:
        raise ValueError("simulation model must contain exactly the 54 TJRC hinge joints")
    return model


class PhysicsSimulation:
    """Single-thread-owned bounded servos; only construction writes qpos.

    ``targets`` and ``qpos_indices`` follow TJRC order: left arm, right arm,
    left hand, right hand. Invalid ready targets reject the entire frame.
    Once a numerical fault is detected, this instance cannot resume stepping.
    """

    def __init__(self, model_path: Path, controller_config: Path, *, object_mesh: Path | None = None):
        self.model = _simulation_model(Path(model_path).resolve(), object_mesh=object_mesh)
        self.data = mujoco.MjData(self.model)
        ids = np.array([self.model.joint(name).id for name in JOINT_NAMES], dtype=np.intp)
        if not np.all(self.model.jnt_type[ids] == mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("all TJRC joints must be scalar hinges")
        self.qpos_indices = self.model.jnt_qposadr[ids].copy()
        self.qpos_indices.flags.writeable = False
        self._ranges = self.model.jnt_range[ids].copy()
        if not np.all(np.isfinite(self._ranges)) or np.any(self._ranges[:, 0] >= self._ranges[:, 1]):
            raise ValueError("TJRC joints must have finite increasing limits")
        force_ranges = self.model.actuator_forcerange
        if not np.all(np.isfinite(force_ranges)) or np.any(force_ranges[:, 0] >= 0) or np.any(force_ranges[:, 1] <= 0):
            raise ValueError("TJRC actuator ranges must be finite and straddle zero")

        posture = load_controller_posture(controller_config)
        self._targets = np.zeros(54)
        for index, region in enumerate((slice(0, 7), slice(7, 14))):
            # Match configuredInitialPosture in the existing C++ controller.
            initial = posture[index] if posture is not None else self._ranges[region].mean(axis=1)
            self._targets[region] = self._validated(initial, region)
        self._targets[14:] = self._validated(self._targets[14:], slice(14, 54))
        self._target_view = self._targets.view()
        self._target_view.flags.writeable = False
        self.data.qpos[self.qpos_indices] = self._targets
        self.data.ctrl[:] = self._targets
        self._fault = None
        mujoco.mj_forward(self.model, self.data)
        self._check_state()

    @property
    def targets(self) -> np.ndarray:
        return self._target_view

    def set_simulation_arm_targets(self, values):
        """Local simulation motion owner; never a hardware command interface."""
        if self._fault is not None:
            raise RuntimeError(self._fault)
        targets = self._validated(values, slice(0, 14))
        self._targets[:14] = targets

    def _validated(self, values, region: slice) -> np.ndarray:
        array = np.asarray(values, dtype=float)
        bounds = self._ranges[region]
        if array.shape != (len(bounds),) or not np.all(np.isfinite(array)):
            raise ValueError(f"joints {region.start}:{region.stop} require {len(bounds)} finite radians")
        invalid = (array < bounds[:, 0] - TARGET_TOLERANCE_RAD) | (array > bounds[:, 1] + TARGET_TOLERANCE_RAD)
        if np.any(invalid):
            index = int(np.flatnonzero(invalid)[0])
            name = JOINT_NAMES[region.start + index]
            raise ValueError(f"{name}: target {array[index]} outside [{bounds[index, 0]}, {bounds[index, 1]}]")
        return np.clip(array, bounds[:, 0], bounds[:, 1])

    def set_targets(self, frame: CommandFrame) -> None:
        if self._fault is not None:
            raise RuntimeError(self._fault)
        if frame.flags & ~(ARMS_READY | LEFT_HAND_READY | RIGHT_HAND_READY):
            raise ValueError("unsupported TJRC ready flags")
        updates = []
        for flag, values, region in (
            (ARMS_READY, frame.left_arm, slice(0, 7)),
            (ARMS_READY, frame.right_arm, slice(7, 14)),
            (LEFT_HAND_READY, frame.left_hand, slice(14, 34)),
            (RIGHT_HAND_READY, frame.right_hand, slice(34, 54)),
        ):
            if frame.flags & flag:
                updates.append((region, self._validated(values, region)))
        for region, values in updates:
            self._targets[region] = values

    def _check_state(self) -> None:
        if self._fault is not None:
            raise RuntimeError(self._fault)
        warnings = [mujoco.mjtWarning(index).name for index, warning in enumerate(self.data.warning) if warning.number]
        if warnings:
            self._fault = "MuJoCo warning: " + ", ".join(warnings)
        elif not np.isfinite(self.data.time) or not all(np.all(np.isfinite(value)) for value in (
            self.data.qpos, self.data.qvel, self.data.qacc, self.data.ctrl,
            self.data.actuator_force, self.data.qfrc_applied, self.data.xfrc_applied,
        )):
            self._fault = "MuJoCo state or applied force is non-finite"
        if self._fault is not None:
            raise RuntimeError(self._fault)

    def step(self) -> None:
        """Advance one timestep with bounded PD actuator forces; never reset."""
        self._check_state()
        self.data.ctrl[:] = self._targets
        previous_time = self.data.time
        try:
            mujoco.mj_step(self.model, self.data)
        except (mujoco.FatalError, ValueError) as error:
            self._fault = f"MuJoCo integration failed: {error}"
            raise RuntimeError(self._fault) from error
        self._check_state()
        if not np.isclose(self.data.time, previous_time + self.model.opt.timestep, rtol=0, atol=1e-12):
            self._fault = "MuJoCo did not advance one timestep (possible numerical reset)"
            raise RuntimeError(self._fault)
