from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "start_pico_bridge.launch.py"
FILTER_LAUNCH_FILE = PACKAGE_ROOT / "launch" / "start_pico_palm_skeleton_filter.launch.py"
LEFT_ARM_CALIBRATION_LAUNCH_FILE = (
    PACKAGE_ROOT / "launch" / "calibrate_pico_left_arm_geometry.launch.py"
)
ARM_CALIBRATION_LAUNCH_FILE = (
    PACKAGE_ROOT / "launch" / "calibrate_pico_arm_geometry.launch.py"
)
TIANJI_TELEOP_LAUNCH_FILE = (
    PACKAGE_ROOT / "launch" / "start_tianji_mujoco_teleop.launch.py"
)


def _load_launch_module():
    spec = spec_from_file_location("start_pico_bridge", LAUNCH_FILE)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_filter_launch_module():
    spec = spec_from_file_location(
        "start_pico_palm_skeleton_filter", FILTER_LAUNCH_FILE
    )
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_left_arm_calibration_launch_module():
    spec = spec_from_file_location(
        "calibrate_pico_left_arm_geometry", LEFT_ARM_CALIBRATION_LAUNCH_FILE
    )
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_arm_calibration_launch_module():
    spec = spec_from_file_location(
        "calibrate_pico_arm_geometry", ARM_CALIBRATION_LAUNCH_FILE
    )
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_tianji_teleop_launch_module():
    spec = spec_from_file_location(
        "start_tianji_mujoco_teleop", TIANJI_TELEOP_LAUNCH_FILE
    )
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_exposes_the_complete_ground_skeleton_contract():
    description = _load_launch_module().generate_launch_description()
    arguments = {
        entity.name for entity in description.entities
        if entity.__class__.__name__ == "DeclareLaunchArgument"
    }

    assert {
        "smpl_raw_topic",
        "smpl_topic",
        "world_reset_topic",
        "smpl_ground_ready_topic",
        "smpl_output_frame",
        "foot_half_extents",
        "stable_window_frames",
        "stability_tolerance_m",
        "require_world_reset",
        "tracking_epoch_state_file",
    } <= arguments


def test_launch_ground_defaults_match_the_runtime_contract():
    module = _load_launch_module()

    assert module.DEFAULT_FOOT_HALF_EXTENTS == "0.115,0.050,0.022"
    assert module.DEFAULT_STABLE_WINDOW_FRAMES == "30"
    assert module.DEFAULT_STABILITY_TOLERANCE_M == "0.02"
    assert module.DEFAULT_REQUIRE_WORLD_RESET == "true"


def test_launch_forwards_ground_ready_topic_to_ground_node_only():
    module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        "host": "127.0.0.1",
        "port": "9999",
        "smpl_raw_topic": "/test/smpl_raw",
        "smpl_topic": "/test/smpl",
        "world_reset_topic": "/test/world_reset",
        "smpl_ground_ready_topic": "/test/ground_ready",
        "smpl_output_frame": "test_ground",
        "foot_half_extents": "0.115,0.050,0.022",
        "stable_window_frames": "30",
        "stability_tolerance_m": "0.02",
        "require_world_reset": "true",
        "tracking_epoch_state_file": "/tmp/pico_tracking_epoch_test",
    })

    nodes = {
        node.node_executable: evaluate_parameters(
            context, node._Node__parameters
        )[0]
        for node in module._setup(context)
    }

    assert "ground_ready_topic" not in nodes["pico_bridge_node"]
    assert (
        nodes["pico_bridge_node"]["tracking_epoch_state_file"]
        == "/tmp/pico_tracking_epoch_test"
    )
    assert (
        nodes["pico_smpl_ground"]["ground_ready_topic"]
        == "/test/ground_ready"
    )


def test_filter_launch_exposes_palm_skeleton_contract():
    description = _load_filter_launch_module().generate_launch_description()
    arguments = {
        entity.name for entity in description.entities
        if entity.__class__.__name__ == "DeclareLaunchArgument"
    }
    assert {
        "start_palm_publishers",
        "left_tcp_artifact",
        "right_tcp_artifact",
        "raw_topic",
        "left_palm_topic",
        "right_palm_topic",
        "left_wrist_pivot_artifact",
        "right_wrist_pivot_artifact",
        "output_topic",
        "ik_output_topic",
        "left_shoulder_local_x_offset_rad",
        "right_shoulder_local_x_offset_rad",
        "status_topic",
        "max_skew_s",
        "min_calibration_samples",
        "palm_cache_size",
        "ratio_max_stretch",
        "require_wrist_pivot_artifact",
        "left_arm_geometry_artifact",
        "require_left_arm_geometry_artifact",
        "right_arm_geometry_artifact",
        "require_right_arm_geometry_artifact",
    } <= arguments


def test_filter_launch_starts_bilateral_runtime_palm_publishers():
    description = _load_filter_launch_module().generate_launch_description()
    nodes = [
        entity for entity in description.entities
        if entity.__class__.__name__ == "Node"
    ]

    assert [node.node_executable for node in nodes].count(
        "pico_palm_tcp_publisher"
    ) == 2
    assert [node.node_executable for node in nodes].count(
        "pico_palm_skeleton_filter"
    ) == 1

    palm_publishers = [
        node for node in nodes
        if node.node_executable == "pico_palm_tcp_publisher"
    ]
    assert all(node.condition is not None for node in palm_publishers)


def test_left_arm_calibration_launch_exposes_solo_contract():
    description = _load_left_arm_calibration_launch_module().generate_launch_description()
    arguments = {
        entity.name
        for entity in description.entities
        if entity.__class__.__name__ == "DeclareLaunchArgument"
    }
    assert {
        "left_tcp_artifact",
        "left_wrist_pivot_artifact",
        "output_dir",
        "max_skew_s",
        "readiness_timeout_s",
        "countdown_s",
        "duration_scale",
        "start_mode",
        "start_left_palm_publisher",
        "raw_topic",
        "palm_topic",
        "tracking_epoch_topic",
        "tracking_epoch_status_topic",
    } <= arguments
    nodes = [
        entity for entity in description.entities
        if entity.__class__.__name__ == "Node"
    ]
    assert [node.node_executable for node in nodes] == [
        "pico_palm_tcp_publisher",
        "pico_left_arm_geometry_calibrator",
    ]
    assert sum(
        entity.__class__.__name__ == "RegisterEventHandler"
        for entity in description.entities
    ) == 2
    assert nodes[0].condition is not None
    assert len(nodes[0]._Node__remappings) == 1
    context = LaunchContext()
    context.launch_configurations["palm_topic"] = "/test/left_palm"
    source, target = nodes[0]._Node__remappings[0]
    assert perform_substitutions(context, source) == "/pico/palm_left"
    assert perform_substitutions(context, target) == "/test/left_palm"


def test_generic_arm_calibration_launch_is_side_parameterized_and_automatic():
    description = _load_arm_calibration_launch_module().generate_launch_description()
    arguments = {
        entity.name
        for entity in description.entities
        if entity.__class__.__name__ == "DeclareLaunchArgument"
    }
    assert {
        "side",
        "tcp_artifact",
        "wrist_pivot_artifact",
        "output_dir",
        "start_palm_publisher",
        "raw_topic",
        "palm_topic",
    } <= arguments
    assert "start_mode" not in arguments
    nodes = [
        entity for entity in description.entities
        if entity.__class__.__name__ == "Node"
    ]
    assert [node.node_executable for node in nodes] == [
        "pico_palm_tcp_publisher",
        "pico_arm_geometry_calibrator",
    ]
    calibrator_arguments = [str(value) for value in nodes[1]._Node__arguments]
    assert "auto" in calibrator_arguments


def test_tianji_teleop_launch_exposes_exact_transport_contract():
    description = _load_tianji_teleop_launch_module().generate_launch_description()
    arguments = {
        entity.name: perform_substitutions(LaunchContext(), entity.default_value)
        for entity in description.entities
        if entity.__class__.__name__ == "DeclareLaunchArgument"
    }

    assert arguments == {
        "skeleton_topic": "/pico/smpl_palm_corrected_ik",
        "status_topic": "/pico/smpl_palm_corrected/status",
        "record_flag_topic": "/pico/record_flag",
        "diagnostics_topic": "/pico/tianji_mujoco_teleop/status",
        "destination_address": "127.0.0.1",
        "destination_port": "15000",
        "cache_capacity": "8",
        "position_retargeting_mode": "robot_arm_segments",
        "robot_arm_reach_scale": "0.95",
        "pico_world_x_offset_m": "0.10",
    }


def test_tianji_teleop_launch_starts_one_parameterized_bridge():
    description = _load_tianji_teleop_launch_module().generate_launch_description()
    nodes = [
        entity
        for entity in description.entities
        if entity.__class__.__name__ == "Node"
    ]

    assert len(nodes) == 1
    assert nodes[0].node_executable == "tianji_mujoco_teleop_bridge"
    assert nodes[0]._Node__node_name == "tianji_mujoco_teleop_bridge"

    context = LaunchContext()
    context.launch_configurations.update({
        "skeleton_topic": "/test/corrected_ik",
        "status_topic": "/test/corrected_status",
        "record_flag_topic": "/test/record_flag",
        "diagnostics_topic": "/test/teleop_status",
        "destination_address": "192.0.2.1",
        "destination_port": "16000",
        "cache_capacity": "12",
        "position_retargeting_mode": "pico_palm",
        "robot_arm_reach_scale": "0.95",
        "pico_world_x_offset_m": "0.18",
    })
    parameters = evaluate_parameters(context, nodes[0]._Node__parameters)[0]
    assert parameters == {
        "skeleton_topic": "/test/corrected_ik",
        "status_topic": "/test/corrected_status",
        "record_flag_topic": "/test/record_flag",
        "diagnostics_topic": "/test/teleop_status",
        "destination_address": "192.0.2.1",
        "destination_port": 16000,
        "cache_capacity": 12,
        "position_retargeting_mode": "pico_palm",
        "robot_arm_reach_scale": 0.95,
        "pico_world_x_offset_m": 0.18,
    }
