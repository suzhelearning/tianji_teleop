from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "skeleton_topic", default_value="/pico/smpl_palm_corrected_ik"
        ),
        DeclareLaunchArgument(
            "status_topic", default_value="/pico/smpl_palm_corrected/status"
        ),
        DeclareLaunchArgument(
            "record_flag_topic", default_value="/pico/record_flag"
        ),
        DeclareLaunchArgument(
            "diagnostics_topic", default_value="/pico/tianji_mujoco_teleop/status"
        ),
        DeclareLaunchArgument("destination_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("destination_port", default_value="15000"),
        DeclareLaunchArgument("cache_capacity", default_value="8"),
        DeclareLaunchArgument(
            "position_retargeting_mode", default_value="robot_arm_segments"
        ),
        DeclareLaunchArgument("robot_arm_reach_scale", default_value="0.95"),
        DeclareLaunchArgument("pico_world_x_offset_m", default_value="0.10"),
    ]
    bridge = Node(
        package="pico_bridge",
        executable="tianji_mujoco_teleop_bridge",
        name="tianji_mujoco_teleop_bridge",
        output="screen",
        parameters=[
            {
                "skeleton_topic": LaunchConfiguration("skeleton_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
                "record_flag_topic": LaunchConfiguration("record_flag_topic"),
                "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
                "destination_address": LaunchConfiguration("destination_address"),
                "destination_port": ParameterValue(
                    LaunchConfiguration("destination_port"), value_type=int
                ),
                "cache_capacity": ParameterValue(
                    LaunchConfiguration("cache_capacity"), value_type=int
                ),
                "position_retargeting_mode": LaunchConfiguration(
                    "position_retargeting_mode"
                ),
                "robot_arm_reach_scale": ParameterValue(
                    LaunchConfiguration("robot_arm_reach_scale"), value_type=float
                ),
                "pico_world_x_offset_m": ParameterValue(
                    LaunchConfiguration("pico_world_x_offset_m"), value_type=float
                ),
            }
        ],
    )
    return LaunchDescription([*arguments, bridge])
