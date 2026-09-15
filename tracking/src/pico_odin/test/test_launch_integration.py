import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "odin_select.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("odin_select_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_runtime_auto_follows_odin_implementation():
    module = _load_launch_module()

    assert module._runtime_enabled("auto", "lite") is True
    assert module._runtime_enabled("auto", "original") is False


def test_runtime_explicit_value_overrides_implementation():
    module = _load_launch_module()

    assert module._runtime_enabled("true", "original") is True
    assert module._runtime_enabled("false", "lite") is False


def test_runtime_rejects_invalid_setting():
    module = _load_launch_module()

    with pytest.raises(RuntimeError, match="enable_pelvis_runtime"):
        module._runtime_enabled("sometimes", "lite")


def test_launch_exposes_runtime_configuration():
    description = _load_launch_module().generate_launch_description()
    arguments = {
        entity.name for entity in description.entities
        if entity.__class__.__name__ == "DeclareLaunchArgument"
    }

    assert {
        "enable_pelvis_runtime",
        "pelvis_extrinsics_file",
        "pelvis_output_topic",
        "raw_odin_highfreq_topic",
        "pico_smpl_fused_topic",
        "world_reset_topic",
        "expected_skeleton_frame",
        "smpl_odin_output_topic",
        "alignment_samples",
        "max_sync_gap_sec",
        "max_pending_age_sec",
        "host_sync_history_sec",
        "max_receive_lag_sec",
        "receive_lag_step_sec",
        "min_receive_lag_correlation",
        "min_receive_lag_pairs",
        "receive_lag_smoothing_alpha",
        "receive_lag_update_period_sec",
        "odin_restart_gap_sec",
        "odin_translation_jump_m",
        "stationary_linear_speed_mps",
    } <= arguments


def test_lite_pelvis_launch_uses_odom_only_channel_profile():
    module = _load_launch_module()
    installed = (
        Path(get_package_share_directory("odin_ros_driver_rev1"))
        / "config"
        / module.LITE_CHANNEL_CONFIG_FILE
    )
    assert installed.is_file()

    register_keys = __import__("yaml").safe_load(
        installed.read_text(encoding="utf-8")
    )["register_keys"]
    assert register_keys == {
        "enable_raw_point": 0,
        "enable_slam_point": 0,
        "enable_image0": 0,
        "enable_image1": 0,
        "enable_imu": 0,
        "enable_odom": 1,
        "post_process": 0,
        "send_image_raw": 0,
        "send_image2_raw": 0,
        "send_cloud_render": 0,
        "send_ess_output": 0,
        "send_depth_pointcloud": 0,
        "enable_reprojection": 0,
        "reprojection_mode": "raw",
    }


def test_lite_driver_disables_fastdds_shared_memory_transport():
    module = _load_launch_module()
    assert module.LITE_ADDITIONAL_ENV == {
        "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"
    }
