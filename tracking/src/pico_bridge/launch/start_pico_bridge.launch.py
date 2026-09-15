# launch/start_pico_bridge.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_FOOT_HALF_EXTENTS = "0.115,0.050,0.022"
DEFAULT_STABLE_WINDOW_FRAMES = "30"
DEFAULT_STABILITY_TOLERANCE_M = "0.02"
DEFAULT_REQUIRE_WORLD_RESET = "true"


def _float_list_argument(context, name, expected_size):
    values = [
        float(value.strip())
        for value in LaunchConfiguration(name).perform(context).split(",")
    ]
    if len(values) != expected_size:
        raise RuntimeError(
            f"{name} must contain {expected_size} comma-separated values"
        )
    return values


def _setup(context, *args, **kwargs):
    bridge = Node(
        package="pico_bridge",
        executable="pico_bridge_node",
        name="pico_bridge",
        output="screen",
        parameters=[{
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "smpl_raw_topic": LaunchConfiguration("smpl_raw_topic"),
            "world_reset_topic": LaunchConfiguration("world_reset_topic"),
            "tracking_epoch_state_file": LaunchConfiguration("tracking_epoch_state_file"),
        }],
    )
    ground = Node(
        package="pico_bridge",
        executable="pico_smpl_ground",
        name="pico_smpl_ground",
        output="screen",
        parameters=[{
            "raw_topic": LaunchConfiguration("smpl_raw_topic"),
            "output_topic": LaunchConfiguration("smpl_topic"),
            "world_reset_topic": LaunchConfiguration("world_reset_topic"),
            "ground_ready_topic": LaunchConfiguration("smpl_ground_ready_topic"),
            "output_frame": LaunchConfiguration("smpl_output_frame"),
            "foot_half_extents": _float_list_argument(
                context, "foot_half_extents", 3
            ),
            "stable_window_frames": ParameterValue(
                LaunchConfiguration("stable_window_frames"), value_type=int
            ),
            "stability_tolerance_m": ParameterValue(
                LaunchConfiguration("stability_tolerance_m"), value_type=float
            ),
            "require_world_reset": ParameterValue(
                LaunchConfiguration("require_world_reset"), value_type=bool
            ),
        }],
    )
    return [bridge, ground]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="9999"),
        DeclareLaunchArgument("smpl_raw_topic", default_value="/pico/smpl_raw"),
        DeclareLaunchArgument("smpl_topic", default_value="/pico/smpl"),
        DeclareLaunchArgument("world_reset_topic", default_value="/pico/world_reset"),
        DeclareLaunchArgument(
            "tracking_epoch_state_file",
            default_value="~/.config/pico_tracker/tracking_epoch",
        ),
        DeclareLaunchArgument(
            "smpl_ground_ready_topic", default_value="/pico/smpl_ground_ready"
        ),
        DeclareLaunchArgument("smpl_output_frame", default_value="pico_ground"),
        DeclareLaunchArgument(
            "foot_half_extents", default_value=DEFAULT_FOOT_HALF_EXTENTS
        ),
        DeclareLaunchArgument(
            "stable_window_frames", default_value=DEFAULT_STABLE_WINDOW_FRAMES
        ),
        DeclareLaunchArgument(
            "stability_tolerance_m", default_value=DEFAULT_STABILITY_TOLERANCE_M
        ),
        DeclareLaunchArgument(
            "require_world_reset", default_value=DEFAULT_REQUIRE_WORLD_RESET
        ),
        OpaqueFunction(function=_setup),
    ])
