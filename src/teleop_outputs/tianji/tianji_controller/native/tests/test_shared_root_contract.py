import importlib.util
from pathlib import Path

import pytest
import yaml
from tianji_runtime import controller_profile
from tianji_runtime.resources import controller_resource

CONTROL = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "shared_root_contract", CONTROL / "scripts/validate_shared_root_contract.py")
validator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validator)


def test_frozen_contract_and_models_match():
    result = validator.validate(controller_profile("qp_ik_pico_shared_root.yaml"))
    assert result["passed"] and not result["motion_authorized"]


@pytest.mark.parametrize("mutation", ["enabled", "hash", "origin", "mix", "sources", "simple_basis", "closure_axis", "closure_frame", "closure_evidence"])
def test_rejects_invalid_contract(tmp_path, mutation):
    profile = yaml.safe_load(controller_profile("qp_ik_pico_shared_root.yaml").read_text())
    contract = yaml.safe_load(controller_profile("shared_root_tjvr_input_contract.yaml").read_text())
    geometry_path = controller_profile("shared_root_robot_geometry.yaml")
    geometry = yaml.safe_load(geometry_path.read_text())
    for kind in ("urdf", "mujoco_xml"):
        geometry["robot_geometry"][kind + "_path"] = str(
            controller_resource(geometry_path, geometry["robot_geometry"][kind + "_path"]))
    if mutation == "enabled":
        profile["spark_shared_root"]["enabled"] = True
    if mutation == "mix":
        profile["spark_shared_root"]["mix"] = .5
    if mutation == "hash":
        geometry["robot_geometry"]["urdf_sha256"] = "0" * 64
    if mutation == "origin":
        contract["tjvr_shared_root_input"]["o_Ct_contract_m"] = [0, 0, 0]
    if mutation == "sources":
        contract["tjvr_shared_root_input"]["allowed_geometry_sources"] = ["unverified_any_source"]
    if mutation == "simple_basis":
        contract["tjvr_shared_root_input"]["simple_geometry_contract"]["tcp_mirror"] = "hardware_verified"
    if mutation == "closure_axis":
        geometry["robot_geometry"]["left"]["T_solver_tcp_to_wrist_center"]["translation_m"] = [0, .1615, 0]
    if mutation == "closure_frame":
        geometry["robot_geometry"]["right"]["elbow_center_frame"] = "Link4_L"
    if mutation == "closure_evidence":
        geometry["robot_geometry"]["closure_validation"]["maximum_wrist_vector_variation_m"] = 1
    cp = tmp_path / "shared_root_tjvr_input_contract.yaml"
    cp.write_text(yaml.safe_dump(contract))
    geometry["robot_geometry"]["tjvr_input_contract_sha256"] = validator.digest(cp)
    (tmp_path / "shared_root_robot_geometry.yaml").write_text(yaml.safe_dump(geometry))
    pp = tmp_path / "experiment.yaml"
    pp.write_text(yaml.safe_dump(profile))
    with pytest.raises((ValueError, AssertionError)):
        validator.validate(pp)
