"""Launch exclusively owned official RGB drivers after exact-mode preflight."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO
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
    nodes = []
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

            actions.append(
                RegisterEventHandler(OnProcessExit(target_action=node, on_exit=on_exit)))
            nodes.append(node)

        # RealSense enumerates all USB devices before selecting its serial.
        # Overlapping enumeration can leave another driver with a partial
        # device and no color sensor. Serialize initialization, not streaming.
        # This log is only an initialization barrier: CameraMonitor still
        # certifies the effective profile and fresh Image/Metadata pairs.
        def start_after_initialization(next_node):
            pending = bytearray()
            started = False

            def on_output(event):
                nonlocal started
                if started:
                    return []
                pending.extend(event.text)
                if b"RealSense Node Is Up!" in pending:
                    started = True
                    return [next_node]
                del pending[:-4096]
                return []

            return on_output

        for previous, following in zip(nodes, nodes[1:]):
            callback = start_after_initialization(following)
            actions.append(RegisterEventHandler(OnProcessIO(
                target_action=previous, on_stdout=callback, on_stderr=callback)))
        if nodes:
            actions.append(nodes[0])

        return actions
    except BaseException:
        locks.close()
        raise


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(config_path("collect_real.json"))),
        OpaqueFunction(function=_camera_nodes),
    ])
