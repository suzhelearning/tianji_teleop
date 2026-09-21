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
        name="pico_left_palm_tcp_publisher_for_geometry",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_left_palm_publisher")),
        arguments=[
            "--side",
            "left",
            "--artifact",
            LaunchConfiguration("left_tcp_artifact"),
        ],
        remappings=[
            ("/pico/palm_left", LaunchConfiguration("palm_topic")),
        ],
    )
    calibrator = Node(
        package="pico_bridge",
        executable="pico_left_arm_geometry_calibrator",
        name="pico_left_arm_geometry_calibrator",
        output="screen",
        emulate_tty=True,
        arguments=[
            "--left-tcp-artifact",
            LaunchConfiguration("left_tcp_artifact"),
            "--left-wrist-pivot-artifact",
            LaunchConfiguration("left_wrist_pivot_artifact"),
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
            LaunchConfiguration("start_mode"),
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "left_tcp_artifact",
                default_value="~/.config/pico_tracker/pico_left_palm_tcp.yaml",
            ),
            DeclareLaunchArgument(
                "left_wrist_pivot_artifact",
                default_value="~/.config/pico_tracker/pico_left_wrist_pivot.yaml",
            ),
            DeclareLaunchArgument("output_dir", default_value=""),
            DeclareLaunchArgument("max_skew_s", default_value="0.03"),
            DeclareLaunchArgument("readiness_timeout_s", default_value="20.0"),
            DeclareLaunchArgument("countdown_s", default_value="3"),
            DeclareLaunchArgument("duration_scale", default_value="1.0"),
            DeclareLaunchArgument("start_mode", default_value="auto"),
            DeclareLaunchArgument("raw_topic", default_value="/pico/smpl_raw"),
            DeclareLaunchArgument("palm_topic", default_value="/pico/palm_left"),
            DeclareLaunchArgument(
                "tracking_epoch_topic", default_value="/pico/tracking_epoch"
            ),
            DeclareLaunchArgument(
                "tracking_epoch_status_topic",
                default_value="/pico/tracking_epoch/status",
            ),
            DeclareLaunchArgument("start_left_palm_publisher", default_value="true"),
            palm_publisher,
            calibrator,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=calibrator,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="left-arm geometry calibration finished"
                            )
                        )
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=palm_publisher,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="left palm publisher stopped during calibration"
                            )
                        )
                    ],
                )
            ),
        ]
    )
