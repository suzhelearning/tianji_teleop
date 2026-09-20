"""The weight trial must not alter limits, solver settings or default enablement."""
from copy import deepcopy
from pathlib import Path

import yaml


def test_palm_priority_changes_only_four_stage2_weights():
    directory = Path(__file__).resolve().parents[1] / "config"
    baseline = yaml.safe_load((directory / "qp_ik_pico_shared_root.yaml").read_text())
    trial = yaml.safe_load((directory / "qp_ik_pico_shared_root_palm_priority.yaml").read_text())
    expected = deepcopy(baseline)
    expected["spark_upper_qpoases"].update(
        stage2_upper_direction_weight=0.05,
        stage2_forearm_direction_weight=0.05,
        stage2_elbow_position_weight=0.25,
        stage2_wrist_position_weight=0.25,
    )
    assert trial == expected
    assert trial["spark_shared_root"]["enabled"] is False
