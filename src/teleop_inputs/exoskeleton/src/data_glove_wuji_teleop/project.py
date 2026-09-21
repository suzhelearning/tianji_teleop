"""独立项目资源定位与启动前验证。"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from .profiles.dataglove.mapping import CalibrationProfile
from .profiles.joint_mapping import load_joint_mapping
from .profiles.wuji_v2.model import MODEL_REVISION, full_joint_name


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SUPPORTED_HANDS = ("left", "right")
SUPPORTED_GENERATIONS = ("v1", "v2")


@dataclass(frozen=True)
class HandResources:
    """某一代际、某一侧的配置与官方模型路径。"""

    generation: str
    hand: str
    calibration: Path
    mapping: Path
    urdf: Path | None = None
    mjcf: Path | None = None

    @property
    def model_path(self) -> Path:
        model = self.urdf if self.generation == "v1" else self.mjcf
        if model is None:
            raise ValueError(f"{self.generation} 模型路径未配置")
        return model


def get_hand_resources(
    hand: str,
    *,
    generation: str = "v1",
    root: Path = PROJECT_ROOT,
) -> HandResources:
    if hand not in SUPPORTED_HANDS:
        raise ValueError(f"hand 必须是 left 或 right，实际为 {hand}")
    if generation not in SUPPORTED_GENERATIONS:
        raise ValueError(
            f"generation 必须是 v1 或 v2，实际为 {generation}"
        )
    root = root.resolve()
    config_root = root / "config" / "hands"
    calibration = root / "config" / "dataglove" / "calibration" / f"{hand}.json"
    description = root / "assets" / "wuji-description"
    if generation == "v1":
        return HandResources(
            generation=generation,
            hand=hand,
            calibration=calibration,
            mapping=config_root / "wuji_v1" / f"{hand}_mapping.json",
            urdf=description / "hand" / "body" / "urdf" / f"{hand}.urdf",
        )
    return HandResources(
        generation=generation,
        hand=hand,
        calibration=calibration,
        mapping=config_root / "wuji_v2" / f"{hand}_mapping.json",
        mjcf=(
            description
            / "hand2"
            / MODEL_REVISION
            / "body"
            / "mjcf"
            / f"{hand}.xml"
        ),
    )


@dataclass(frozen=True)
class ValidationReport:
    """一次独立项目验证的可检查结果。"""

    generation: str
    calibrated_hand: str | None
    mapping_count: int
    mesh_count: int
    mujoco_joint_count: int | None
    mujoco_actuator_count: int | None


def validate_project(
    *,
    root: Path = PROJECT_ROOT,
    generation: str = "v1",
    hand: str = "left",
    load_mujoco: bool = False,
    require_calibration: bool = True,
) -> ValidationReport:
    """验证指定代际和侧别的标定、映射与模型资源。"""

    root = root.resolve()
    resources = get_hand_resources(
        hand,
        generation=generation,
        root=root,
    )
    calibration_path = resources.calibration
    model_path = resources.model_path
    if not model_path.is_file():
        raise FileNotFoundError(
            f"新项目缺少 Wuji {generation} {hand} 模型资源：{model_path}；"
            f"请补齐 {root / 'assets/wuji-description'} 内的官方模型和网格"
        )

    calibrated_hand: str | None = None
    if calibration_path.is_file():
        profile = CalibrationProfile.load(calibration_path)
        if profile.hand != hand:
            raise ValueError(
                f"{hand} 标定文件的 hand 实际为 {profile.hand}"
            )
        calibrated_hand = profile.hand
    elif require_calibration:
        raise FileNotFoundError(
            f"缺少 {hand} 标定文件：{calibration_path}；"
            f"请运行 pixi run calibrate-{hand}"
        )

    mapping = load_joint_mapping(
        resources.mapping,
        expected_hand=hand,
        expected_generation=generation,
    )
    target_names = [rule.joint_name for rule in mapping]

    model_root = ET.parse(model_path).getroot()
    mesh_paths: set[Path] = set()
    mesh_base = model_path.parent
    if generation == "v2":
        compiler = model_root.find("compiler")
        if compiler is not None:
            mesh_base /= compiler.get("meshdir", "")
    for mesh in model_root.findall(".//mesh"):
        filename = mesh.get("filename") or mesh.get("file")
        if not filename:
            raise ValueError(f"模型 mesh 缺少文件路径：{model_path}")
        mesh_path = (mesh_base / filename).resolve()
        if not mesh_path.is_file():
            raise FileNotFoundError(f"缺少 MuJoCo 网格：{mesh_path}")
        mesh_paths.add(mesh_path)

    mujoco_joint_count: int | None = None
    mujoco_actuator_count: int | None = None
    if load_mujoco:
        if generation == "v1":
            from .adapters.simulation.mujoco_wuji_v1 import _load_model

            model, _, joint_indices = _load_model(model_path)
            full_names = [f"{hand}_{name}" for name in target_names]
            missing = [
                name for name in full_names if name not in joint_indices
            ]
            mujoco_joint_count = len(joint_indices)
            mujoco_actuator_count = model.nu
        else:
            import mujoco

            model = mujoco.MjModel.from_xml_path(str(model_path))
            full_names = [
                full_joint_name(hand, name) for name in target_names
            ]
            joint_ids = [
                mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    name,
                )
                for name in full_names
            ]
            missing = [
                name
                for name, joint_id in zip(full_names, joint_ids)
                if joint_id < 0
            ]
            actuator_joint_ids = {
                int(model.actuator_trnid[index, 0])
                for index in range(model.nu)
            }
            missing_actuators = [
                name
                for name, joint_id in zip(full_names, joint_ids)
                if joint_id >= 0 and joint_id not in actuator_joint_ids
            ]
            if missing_actuators:
                raise ValueError(
                    f"MuJoCo 模型关节缺少执行器：{missing_actuators}"
                )
            mujoco_joint_count = model.njnt
            mujoco_actuator_count = model.nu
        if missing:
            raise ValueError(f"MuJoCo 模型缺少映射关节：{missing}")

    return ValidationReport(
        generation=generation,
        calibrated_hand=calibrated_hand,
        mapping_count=len(mapping),
        mesh_count=len(mesh_paths),
        mujoco_joint_count=mujoco_joint_count,
        mujoco_actuator_count=mujoco_actuator_count,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="验证独立项目的配置和 MuJoCo 资源")
    parser.add_argument(
        "--generation",
        choices=(*SUPPORTED_GENERATIONS, "all"),
        default="all",
        help="验证 v1、v2 或全部代际资源（默认 all）",
    )
    parser.add_argument(
        "--hand",
        choices=(*SUPPORTED_HANDS, "both"),
        default="both",
        help="验证左手、右手或双手资源（默认 both）",
    )
    parser.add_argument(
        "--load-mujoco",
        action="store_true",
        help="实际加载 MuJoCo 模型（无需打开窗口）",
    )
    parser.add_argument(
        "--allow-missing-calibration",
        action="store_true",
        help="允许尚未生成某侧用户标定，但仍验证映射和模型",
    )
    args = parser.parse_args()
    hands = SUPPORTED_HANDS if args.hand == "both" else (args.hand,)
    generations = (
        SUPPORTED_GENERATIONS
        if args.generation == "all"
        else (args.generation,)
    )
    for generation in generations:
        for hand in hands:
            report = validate_project(
                generation=generation,
                hand=hand,
                load_mujoco=args.load_mujoco,
                require_calibration=not args.allow_missing_calibration,
            )
            print(
                "项目资源有效："
                f"generation={generation} "
                f"hand={hand} "
                f"calibrated={report.calibrated_hand is not None} "
                f"mapping={report.mapping_count} "
                f"meshes={report.mesh_count} "
                f"mujoco_joints={report.mujoco_joint_count} "
                f"mujoco_actuators={report.mujoco_actuator_count}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
