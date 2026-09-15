"""URDF 之后的公共 MANO/Wuji 配置，只按手侧选择，不依赖设备档案。"""

from __future__ import annotations

from pathlib import Path

import yaml

from ...project import PROJECT_ROOT


DEFAULT_MANO_MORPHOLOGY = (
    PROJECT_ROOT / "config/retargeting/wuji_v2_right_mano_morphology.json"
)


def official_config_path(hand: str) -> Path:
    if hand not in ("left", "right"):
        raise ValueError("Wuji 重映射手侧必须为 left/right")
    return PROJECT_ROOT / "config/retargeting/official" / f"{hand}.yaml"


def _read_config(path: Path) -> dict:
    if not path.is_file():
        raise ValueError(f"公共重映射配置不存在：{path}")
    try:
        with path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f"公共重映射配置格式无效：{path}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"公共重映射配置根节点必须是对象：{path}")
    return data


def validate_retargeting_resources(
    hand: str,
    *,
    mano_morphology: str | Path = DEFAULT_MANO_MORPHOLOGY,
    wuji_config: str | Path | None = None,
) -> Path:
    """在启动求解或设备控制前校验公共配置，返回实际官方 YAML 路径。"""
    default_config = official_config_path(hand)
    config = default_config if wuji_config is None else Path(wuji_config).resolve()
    morphology = _read_config(Path(mano_morphology))
    if morphology.get("hand") != "right":
        raise ValueError("mano_morphology 必须是现有规范右手骨架")
    wuji = _read_config(config)
    for key in ("hand", "hand_side"):
        if key in wuji and wuji[key] != hand:
            raise ValueError(f"wuji_config 的 {key} 与所选手侧不一致")
    optimizer = wuji.get("optimizer")
    naming = optimizer.get("link_naming") if isinstance(optimizer, dict) else None
    prefix = naming.get("prefix") if isinstance(naming, dict) else None
    prefix_hand = {"l_": "left", "r_": "right"}.get(prefix) if isinstance(prefix, str) else None
    if prefix_hand is not None and prefix_hand != hand:
        raise ValueError(f"wuji_config 的 optimizer.link_naming.prefix={prefix} 与所选手侧不一致")
    return config
