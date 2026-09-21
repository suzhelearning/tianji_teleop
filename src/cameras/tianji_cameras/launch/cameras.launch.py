"""Launch exclusively owned official RGB drivers after exact-mode preflight."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tianji_runtime import config_path
from tianji_cameras.preflight import (
    CameraLocks, TARGET_FORMAT, TARGET_PROFILE, load_camera_selection, validate_devices,
)


def _camera_nodes(context):
    selection = load_camera_selection(LaunchConfiguration("config").perform(context))
    # Acquire every serial before enumerating or starting the first driver.
    locks = CameraLocks(selection.values())
    context.extend_globals({"tianji_camera_locks": locks})
    actions = []
    try:
        profiles = validate_devices(selection)
        for role, profile in profiles.items():
            channel = "depth_module" if profile.uses_depth_module_color else "rgb_camera"
            node = Node(
                package="realsense2_camera", executable="realsense2_camera_node",
                namespace="cameras", name=role, output="screen", respawn=False,
                parameters=[{
                    "camera_name": role,
                    "serial_no": f"_{profile.serial}",
                    "enable_color": True, "enable_depth": False,
                    "enable_infra": False, "enable_infra1": False, "enable_infra2": False,
                    "enable_gyro": False, "enable_accel": False,
                    "enable_sync": False, "initial_reset": False,
                    "wait_for_device_timeout": 10.0,
                    "pointcloud.enable": False, "align_depth.enable": False,
                    "color_qos": "SENSOR_DATA", "color_info_qos": "SENSOR_DATA",
                    f"{channel}.color_profile": TARGET_PROFILE,
                    f"{channel}.color_format": TARGET_FORMAT,
                }])

            def on_exit(event, context, serial=profile.serial, name=role):
                # The lock cannot be released by the shutdown request: only the
                # process-exit event proves the pipeline owner has gone away.
                locks.release(serial)
                return [EmitEvent(event=Shutdown(
                    reason=f"camera {name} driver exited with code {event.returncode}"))]

            actions.extend([
                RegisterEventHandler(OnProcessExit(target_action=node, on_exit=on_exit)),
                node,
            ])

        return actions
    except BaseException:
        locks.close()
        raise


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(config_path("collect_real.json"))),
        OpaqueFunction(function=_camera_nodes),
    ])
