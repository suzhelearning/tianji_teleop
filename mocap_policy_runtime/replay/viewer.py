"""Read-only reference geometry: no IK, actuator stepping, or hardware output."""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from control.home_config import load_controller_posture

from ..types import TargetFrame
from ..integration.targets import model_wrist_pose

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = ROOT / "control/models/marvin_m6_wuji2.xml"
DEFAULT_CONFIG = ROOT / "control/config/qp_ik_pico_teleop.yaml"
EDGES = tuple((0 if joint == 0 else 4 * finger + joint, 4 * finger + joint + 1)
              for finger in range(5) for joint in range(4))




class ReferenceScene:
    def __init__(self, model_path: Path = DEFAULT_MODEL, config_path: Path = DEFAULT_CONFIG,
                 object_mesh: Path | None = None, *, live_overlay: bool = False):
        model_path = Path(model_path).resolve()
        root = ET.parse(model_path).getroot()
        compiler = root.find("compiler")
        if compiler is not None:
            for field in ("meshdir", "texturedir"):
                if compiler.get(field):
                    compiler.set(field, str((model_path.parent / compiler.get(field)).resolve()))
        self._object_bodies = {}
        if object_mesh is not None:
            object_mesh = Path(object_mesh).resolve(strict=True)
            asset = root.find("asset")
            if asset is None:
                asset = ET.SubElement(root, "asset")
            ET.SubElement(asset, "mesh", name="mocap_hammer_mesh", file=str(object_mesh))
            objects = {"hammer": "0.1 0.9 0.25 0.8"}
            if live_overlay:
                objects["live_hammer"] = "1 0.35 0.05 0.8"
            for name, color in objects.items():
                body_name = f"mocap_{name}"
                body = ET.SubElement(root.find("worldbody"), "body", name=body_name, mocap="true")
                ET.SubElement(body, "geom", type="mesh", mesh="mocap_hammer_mesh",
                              rgba=color, contype="0", conaffinity="0")
                self._object_bodies[name] = body_name
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        posture = load_controller_posture(config_path)
        self._hand_addresses = {}
        self._hand_ranges = {}
        self._geoms = {}
        from sim.physics import JOINT_NAMES
        for side, arm_offset, hand_offset in (("left", 0, 14), ("right", 7, 34)):
            arm_ids = [self.model.joint(name).id for name in JOINT_NAMES[arm_offset:arm_offset + 7]]
            initial = posture[arm_offset // 7] if posture is not None else None
            self.data.qpos[self.model.jnt_qposadr[arm_ids]] = (
                np.asarray(initial) if initial is not None else self.model.jnt_range[arm_ids].mean(axis=1))
            hand_ids = [self.model.joint(name).id for name in JOINT_NAMES[hand_offset:hand_offset + 20]]
            self._hand_addresses[side] = self.model.jnt_qposadr[hand_ids]
            self._hand_ranges[side] = self.model.jnt_range[hand_ids]
            self._geoms[side] = [index for index in range(self.model.ngeom)
                                 if self.model.geom_type[index] == mujoco.mjtGeom.mjGEOM_MESH
                                 and self.model.mesh(int(self.model.geom_dataid[index])).name.startswith(f"wuji2_{side[0]}_")
                                 and self.model.mesh(int(self.model.geom_dataid[index])).name != f"wuji2_{side[0]}_mount"]
        mujoco.mj_forward(self.model, self.data)
        self.frame = None
        self._visible_alpha = self.model.geom_rgba[:, 3].copy()

    def update(self, frame: TargetFrame) -> None:
        # This is a reference-only surface. A static robot base in training
        # coordinates would obscure the hand and imply a nonexistent alignment.
        self.model.geom_rgba[:, 3] = 0.0
        for side in frame.wrist_poses:
            indices = self._geoms[side]
            self.model.geom_rgba[indices, 3] = self._visible_alpha[indices]
        for side, joints in frame.hand_joints.items():
            values = np.asarray(joints, dtype=float)
            limits = self._hand_ranges[side]
            if values.shape != (20,) or not np.isfinite(values).all():
                raise ValueError(f"invalid {side} reference hand joints")
            if np.any(values < limits[:, 0] - 1e-6) or np.any(values > limits[:, 1] + 1e-6):
                raise ValueError(f"{side} reference hand joints exceed model limits")
            self.data.qpos[self._hand_addresses[side]] = np.clip(values, limits[:, 0], limits[:, 1])
        for name, body_name in self._object_bodies.items():
            if name in frame.object_poses:
                indices = self.model.geom_bodyid == self.model.body(body_name).id
                self.model.geom_rgba[indices, 3] = self._visible_alpha[indices]
                pose = frame.object_poses[name]
                mocap_id = self.model.body(body_name).mocapid[0]
                self.data.mocap_pos[mocap_id] = pose[:3]
                self.data.mocap_quat[mocap_id] = np.roll(pose[3:], 1)
        mujoco.mj_forward(self.model, self.data)
        for side, target in frame.wrist_poses.items():
            if frame.space != "mocap_wrist":
                continue
            current = model_wrist_pose(self.model, self.data, side)
            rotation = Rotation.from_quat(target[3:]) * Rotation.from_quat(current[3:]).inv()
            indices = self._geoms[side]
            self.data.geom_xpos[indices] = rotation.apply(self.data.geom_xpos[indices] - current[:3]) + target[:3]
            self.data.geom_xmat[indices] = (rotation.as_matrix() @ self.data.geom_xmat[indices].reshape(-1, 3, 3)).reshape(-1, 9)
        self.frame = frame

    def draw(self, scene) -> None:
        if self.frame is None:
            return
        for pose in (*self.frame.wrist_poses.values(), *self.frame.object_poses.values()):
            rotation = Rotation.from_quat(pose[3:]).as_matrix()
            for axis, color in enumerate(((1, 0.1, 0.1, 1), (0.1, 1, 0.1, 1), (0.1, 0.3, 1, 1))):
                self._line(scene, pose[:3], pose[:3] + 0.06 * rotation[:, axis], color, 0.002)
        for side, points in self.frame.hand_keypoints.items():
            if side not in self.frame.wrist_poses:
                continue
            pose = self.frame.wrist_poses[side]
            world = Rotation.from_quat(pose[3:]).apply(points) + pose[:3]
            for first, second in EDGES:
                self._line(scene, world[first], world[second], (0.9, 0.6, 0.15, 1), 0.0015)

    @staticmethod
    def _line(scene, first, second, rgba, width):
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                           np.zeros(3), np.eye(3).reshape(-1), np.asarray(rgba, dtype=np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, width, first, second)
        scene.ngeom += 1

    def camera(self):
        camera = mujoco.MjvCamera()
        camera.distance = 2.2
        camera.azimuth = 135
        camera.elevation = -25
        camera.lookat[:] = (0, 0, 0.85)
        if self.frame is not None and self.frame.wrist_poses:
            camera.lookat[:] = np.mean([pose[:3] for pose in self.frame.wrist_poses.values()], axis=0)
            camera.distance = 1.1
        return camera

    def snapshot(self, path: Path) -> None:
        from PIL import Image
        with mujoco.Renderer(self.model, height=480, width=640) as renderer:
            renderer.update_scene(self.data, camera=self.camera())
            self.draw(renderer.scene)
            Image.fromarray(renderer.render()).save(path)
