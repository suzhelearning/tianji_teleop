"""Path resolution contract: one authority, explicit failures, no guessing.

These are the rules every entry point relies on, so they are asserted directly
rather than through a package that happens to use them.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tianji_runtime import resources  # noqa: E402


@pytest.fixture()
def workspace(monkeypatch, tmp_path):
    """A minimal explicit checkout with an isolated SPD overlay."""
    (tmp_path / "pixi.toml").write_text("[workspace]\n")
    monkeypatch.setenv("TIANJI_WORKSPACE", str(tmp_path))
    monkeypatch.setenv("TIANJI_ENVIRONMENT", "spd")
    # package_share also consults the live ament prefix path; clearing it keeps
    # this test from passing because the real workspace happens to be built.
    monkeypatch.setenv("AMENT_PREFIX_PATH", "")
    return tmp_path


def test_workspace_honours_the_declared_root(workspace):
    assert resources.workspace() == workspace


def test_workspace_requires_explicit_absolute_root(workspace, monkeypatch):
    monkeypatch.chdir(workspace)
    monkeypatch.delenv("TIANJI_WORKSPACE")
    with pytest.raises(resources.ResourceNotFound):
        resources.workspace()
    monkeypatch.setenv("TIANJI_WORKSPACE", ".")
    with pytest.raises(resources.ResourceNotFound):
        resources.workspace()


def test_workspace_rejects_a_directory_without_the_marker(monkeypatch, tmp_path):
    (tmp_path / "not-a-workspace").mkdir()
    monkeypatch.setenv("TIANJI_WORKSPACE", str(tmp_path / "not-a-workspace"))
    with pytest.raises(resources.ResourceNotFound) as error:
        resources.workspace()
    assert "pixi.toml" in str(error.value)


def test_native_executable_rejects_paths(workspace):
    with pytest.raises(ValueError, match="basename"):
        resources.native_executable("bin/tool")
    with pytest.raises(ValueError, match="basename"):
        resources.native_executable("..")
    with pytest.raises(ValueError):
        resources.native_executable("")


def test_native_executable_finds_an_installed_binary(workspace):
    target = workspace / "install" / "spd" / "bin" / "tianji_qp_ik_viewer"
    target.parent.mkdir(parents=True)
    target.write_text("#!/bin/sh\n")
    target.chmod(0o755)
    assert resources.native_executable("tianji_qp_ik_viewer") == target


def test_native_executable_requires_the_execute_bit(workspace):
    target = workspace / "install" / "spd" / "bin" / "pico2_dls_worker"
    target.parent.mkdir(parents=True)
    target.write_text("not executable\n")
    target.chmod(0o644)
    with pytest.raises(resources.ResourceNotFound):
        resources.native_executable("pico2_dls_worker")


def test_controller_profile_prefers_the_installed_copy(workspace):
    installed = (workspace / "install" / "spd" / "share" / "tianji_controller"
                 / "config" / "qp_ik_pico_shared_root_dls.yaml")
    installed.parent.mkdir(parents=True)
    installed.write_text("controller: {}\n")
    assert resources.controller_profile("qp_ik_pico_shared_root_dls.yaml") == installed


def test_controller_profile_falls_back_to_the_source_tree(workspace):
    source = (workspace / "src" / "teleop_outputs" / "tianji" / "tianji_controller"
              / "native" / "config" / "qp_ik_pico_shared_root_dls.yaml")
    source.parent.mkdir(parents=True)
    source.write_text("controller: {}\n")
    assert resources.controller_profile("qp_ik_pico_shared_root_dls.yaml") == source


def test_controller_profile_rejects_a_path(workspace):
    with pytest.raises(ValueError, match="basename"):
        resources.controller_profile("config/qp_ik.yaml")


def test_package_share_reports_a_missing_member(workspace):
    share = workspace / "install" / "spd" / "share" / "tianji_description"
    (share / "models").mkdir(parents=True)
    with pytest.raises(resources.ResourceNotFound) as error:
        resources.package_share("tianji_description", "models", "absent.xml")
    assert "absent.xml" in str(error.value)


def test_controller_resource_preserves_custom_and_explicit_paths(workspace):
    profile = workspace / "custom" / "profile.yaml"
    profile.parent.mkdir()
    model = profile.parent / "robot.urdf"
    model.write_text("custom robot")
    assert resources.controller_resource(profile, "robot.urdf") == model
    assert resources.controller_resource(profile, str(model)) == model
    missing = profile.parent / "missing.urdf"
    assert resources.controller_resource(profile, "missing.urdf") == missing
    assert resources.controller_resource(profile, str(missing)) == missing


def test_controller_resource_uses_local_description_before_installed(workspace):
    profile = workspace / "src/teleop_outputs/tianji/tianji_controller/native/config/profile.yaml"
    profile.parent.mkdir(parents=True)
    local = workspace / "src/teleop_outputs/tianji/tianji_description/models/custom.urdf"
    local.parent.mkdir(parents=True)
    local.write_text("local description")
    reference = "../../../tianji_description/models/custom.urdf"
    assert resources.controller_resource(profile, reference) == local
    local.unlink()
    with pytest.raises(resources.ResourceNotFound):
        resources.controller_resource(profile, reference)
    installed = workspace / "install/spd/share/tianji_description/models/custom.urdf"
    installed.parent.mkdir(parents=True)
    installed.write_text("installed description")
    assert resources.controller_resource(profile, reference) == installed
