"""Launch the command-only Manus -> Wuji Hand2 ROS pipeline."""

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler,
    SetEnvironmentVariable, UnsetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from manus_bridge.paths import calibration_dir


def _required_process_exited(event, context):
    if context.is_shutdown:
        return
    # LaunchService turns exceptions into a nonzero result and shuts down all
    # launch-owned processes. A Shutdown event alone would report success.
    raise RuntimeError(
        f"Required Manus process {event.process_name} exited unexpectedly "
        f"(exit code {event.returncode})"
    )


def _required_processes(nodes):
    # Register first, so even an immediate import/init failure is fatal.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=_required_process_exited,
    )) for node in nodes]
    return [*handlers, *nodes]



def _setup(context):
    user = LaunchConfiguration("user").perform(context)
    directory = calibration_dir(user)
    start = LaunchConfiguration("start_acquisition").perform(context).lower()
    if start not in ("true", "false"):
        raise ValueError("start_acquisition must be true or false")
    age = float(LaunchConfiguration("max_source_age_s").perform(context))
    if not math.isfinite(age) or age <= 0:
        raise ValueError("max_source_age_s must be finite and positive")
    common = {"max_source_age_s": age}
    nodes = [
        Node(package="manus_bridge", executable="manus_adapter",
             output="screen", parameters=[common]),
        Node(package="manus_bridge", executable="manus_hand2_retarget",
             output="screen", parameters=[common]),
    ]
    if start == "true":
        nodes.append(Node(
            package="manus_bridge", executable="manus_data_publisher",
            output="screen", parameters=[{
                **common,
                "calibration.directory": str(directory),
                "calibration.left_file": f"{user}LeftMetaglovePro.mcal",
                "calibration.right_file": f"{user}RightMetaglovePro.mcal",
            }],
        ))
    return _required_processes(nodes)


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", EnvironmentVariable("ROS_DOMAIN_ID", default_value="120")),
        UnsetEnvironmentVariable("ROS_LOCALHOST_ONLY"),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        DeclareLaunchArgument("user", description="Wearer with a complete profiles/<user>/manus calibration pair"),
        DeclareLaunchArgument("start_acquisition", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("max_source_age_s", default_value="0.25"),
        OpaqueFunction(function=_setup),
    ])
