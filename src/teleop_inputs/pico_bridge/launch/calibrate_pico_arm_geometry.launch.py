from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    palm_publisher = Node(
        package="pico_bridge",
        executable="pico_palm_tcp_publisher",
        name="pico_palm_tcp_publisher_for_arm_geometry",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_palm_publisher")),
        arguments=[
            "--side",
            LaunchConfiguration("side"),
            "--artifact",
            LaunchConfiguration("tcp_artifact"),
        ],
        remappings=[
            ("/pico/palm_left", LaunchConfiguration("palm_topic")),
            ("/pico/palm_right", LaunchConfiguration("palm_topic")),
        ],
    )
    calibrator = Node(
        package="pico_bridge",
        executable="pico_arm_geometry_calibrator",
        name="pico_arm_geometry_calibrator",
        output="screen",
        arguments=[
            "--side",
            LaunchConfiguration("side"),
            "--tcp-artifact",
            LaunchConfiguration("tcp_artifact"),
            "--wrist-pivot-artifact",
            LaunchConfiguration("wrist_pivot_artifact"),
            "--output-dir",
            LaunchConfiguration("output_dir"),
            "--max-skew-s",
            LaunchConfiguration("max_skew_s"),
            "--readiness-timeout-s",
            LaunchConfiguration("readiness_timeout_s"),
            "--countdown-s",
            LaunchConfiguration("countdown_s"),
            "--duration-scale",
            LaunchConfiguration("duration_scale"),
            "--start-mode",
            "auto",
            "--raw-topic",
            LaunchConfiguration("raw_topic"),
            "--palm-topic",
            LaunchConfiguration("palm_topic"),
            "--tracking-epoch-topic",
            LaunchConfiguration("tracking_epoch_topic"),
            "--tracking-epoch-status-topic",
            LaunchConfiguration("tracking_epoch_status_topic"),
        ],
    )
    shutdown_when_calibrator_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrator,
            on_exit=[EmitEvent(event=Shutdown(reason="arm geometry calibration finished"))],
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("side", choices=("left", "right")),
            DeclareLaunchArgument("tcp_artifact"),
            DeclareLaunchArgument("wrist_pivot_artifact"),
            DeclareLaunchArgument("output_dir", default_value=""),
            DeclareLaunchArgument("max_skew_s", default_value="0.03"),
            DeclareLaunchArgument("readiness_timeout_s", default_value="20.0"),
            DeclareLaunchArgument("countdown_s", default_value="3"),
            DeclareLaunchArgument("duration_scale", default_value="1.0"),
            DeclareLaunchArgument("raw_topic", default_value="/pico/smpl_raw"),
            DeclareLaunchArgument("palm_topic"),
            DeclareLaunchArgument(
                "tracking_epoch_topic", default_value="/pico/tracking_epoch"
            ),
            DeclareLaunchArgument(
                "tracking_epoch_status_topic",
                default_value="/pico/tracking_epoch/status",
            ),
            DeclareLaunchArgument("start_palm_publisher", default_value="true"),
            palm_publisher,
            calibrator,
            shutdown_when_calibrator_finishes,
        ]
    )
