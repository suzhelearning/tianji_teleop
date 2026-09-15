from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_palm_publishers", default_value="true"),
            DeclareLaunchArgument(
                "left_tcp_artifact",
                default_value="~/.config/pico_tracker/pico_left_palm_tcp.yaml",
            ),
            DeclareLaunchArgument(
                "right_tcp_artifact",
                default_value="~/.config/pico_tracker/pico_right_palm_tcp.yaml",
            ),
            DeclareLaunchArgument("raw_topic", default_value="/pico/smpl_raw"),
            DeclareLaunchArgument("left_palm_topic", default_value="/pico/palm_left"),
            DeclareLaunchArgument("right_palm_topic", default_value="/pico/palm_right"),
            DeclareLaunchArgument(
                "left_wrist_pivot_artifact",
                default_value="~/.config/pico_tracker/pico_left_wrist_pivot.yaml",
            ),
            DeclareLaunchArgument(
                "right_wrist_pivot_artifact",
                default_value="~/.config/pico_tracker/pico_right_wrist_pivot.yaml",
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/pico/smpl_palm_corrected"
            ),
            DeclareLaunchArgument(
                "ik_output_topic", default_value="/pico/smpl_palm_corrected_ik"
            ),
            DeclareLaunchArgument(
                "left_shoulder_local_x_offset_rad",
                default_value="1.5707963267948966",
            ),
            DeclareLaunchArgument(
                "right_shoulder_local_x_offset_rad",
                default_value="-1.5707963267948966",
            ),
            DeclareLaunchArgument(
                "status_topic", default_value="/pico/smpl_palm_corrected/status"
            ),
            DeclareLaunchArgument(
                "tracking_epoch_topic", default_value="/pico/tracking_epoch"
            ),
            DeclareLaunchArgument(
                "tracking_epoch_status_topic",
                default_value="/pico/tracking_epoch/status",
            ),
            DeclareLaunchArgument("max_skew_s", default_value="0.03"),
            DeclareLaunchArgument("min_calibration_samples", default_value="60"),
            DeclareLaunchArgument("palm_cache_size", default_value="120"),
            DeclareLaunchArgument("ratio_max_stretch", default_value="0.995"),
            DeclareLaunchArgument("require_wrist_pivot_artifact", default_value="false"),
            DeclareLaunchArgument("left_arm_geometry_artifact", default_value=""),
            DeclareLaunchArgument(
                "require_left_arm_geometry_artifact", default_value="false"
            ),
            DeclareLaunchArgument("right_arm_geometry_artifact", default_value=""),
            DeclareLaunchArgument(
                "require_right_arm_geometry_artifact", default_value="false"
            ),
            Node(
                package="pico_bridge",
                executable="pico_palm_tcp_publisher",
                name="pico_left_palm_tcp_publisher",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_palm_publishers")),
                arguments=[
                    "--side",
                    "left",
                    "--artifact",
                    LaunchConfiguration("left_tcp_artifact"),
                ],
            ),
            Node(
                package="pico_bridge",
                executable="pico_palm_tcp_publisher",
                name="pico_right_palm_tcp_publisher",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_palm_publishers")),
                arguments=[
                    "--side",
                    "right",
                    "--artifact",
                    LaunchConfiguration("right_tcp_artifact"),
                ],
            ),
            Node(
                package="pico_bridge",
                executable="pico_palm_skeleton_filter",
                name="pico_palm_skeleton_filter",
                output="screen",
                parameters=[
                    {
                        "raw_topic": LaunchConfiguration("raw_topic"),
                        "left_palm_topic": LaunchConfiguration("left_palm_topic"),
                        "right_palm_topic": LaunchConfiguration("right_palm_topic"),
                        "left_tcp_artifact": LaunchConfiguration("left_tcp_artifact"),
                        "right_tcp_artifact": LaunchConfiguration("right_tcp_artifact"),
                        "left_wrist_pivot_artifact": LaunchConfiguration(
                            "left_wrist_pivot_artifact"
                        ),
                        "right_wrist_pivot_artifact": LaunchConfiguration(
                            "right_wrist_pivot_artifact"
                        ),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "ik_output_topic": LaunchConfiguration("ik_output_topic"),
                        "left_shoulder_local_x_offset_rad": ParameterValue(
                            LaunchConfiguration("left_shoulder_local_x_offset_rad"),
                            value_type=float,
                        ),
                        "right_shoulder_local_x_offset_rad": ParameterValue(
                            LaunchConfiguration("right_shoulder_local_x_offset_rad"),
                            value_type=float,
                        ),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "tracking_epoch_topic": LaunchConfiguration(
                            "tracking_epoch_topic"
                        ),
                        "tracking_epoch_status_topic": LaunchConfiguration(
                            "tracking_epoch_status_topic"
                        ),
                        "max_skew_s": ParameterValue(
                            LaunchConfiguration("max_skew_s"), value_type=float
                        ),
                        "min_calibration_samples": ParameterValue(
                            LaunchConfiguration("min_calibration_samples"), value_type=int
                        ),
                        "palm_cache_size": ParameterValue(
                            LaunchConfiguration("palm_cache_size"), value_type=int
                        ),
                        "ratio_max_stretch": ParameterValue(
                            LaunchConfiguration("ratio_max_stretch"), value_type=float
                        ),
                        "require_wrist_pivot_artifact": ParameterValue(
                            LaunchConfiguration("require_wrist_pivot_artifact"),
                            value_type=bool,
                        ),
                        "left_arm_geometry_artifact": LaunchConfiguration(
                            "left_arm_geometry_artifact"
                        ),
                        "require_left_arm_geometry_artifact": ParameterValue(
                            LaunchConfiguration(
                                "require_left_arm_geometry_artifact"
                            ),
                            value_type=bool,
                        ),
                        "right_arm_geometry_artifact": LaunchConfiguration(
                            "right_arm_geometry_artifact"
                        ),
                        "require_right_arm_geometry_artifact": ParameterValue(
                            LaunchConfiguration(
                                "require_right_arm_geometry_artifact"
                            ),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
