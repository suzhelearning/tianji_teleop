import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_LEFT_PORT = "/dev/ttyUSB0"
DEFAULT_RIGHT_PORT = "/dev/ttyUSB1"
DEFAULT_MOUNT_QUATERNION = "0,0,0,1"
IMU_REMAPPINGS = [
    ("im900/left_foot/imuData_raw", "/imu/left_feet"),
    ("im900/right_foot/imuData_raw", "/imu/right_feet"),
    ("im900/left_foot/ready", "/imu/left_feet/ready"),
    ("im900/right_foot/ready", "/imu/right_feet/ready"),
]


def _quaternion_argument(context, name):
    values = [float(value.strip()) for value in LaunchConfiguration(name).perform(context).split(",")]
    if len(values) != 4:
        raise RuntimeError(f"{name} must contain four comma-separated values")
    return values


def _setup(context, *args, **kwargs):
    config = os.path.join(
        get_package_share_directory("pico_bridge"), "config", "imu900_feet.yaml"
    )
    driver = Node(
        package="imu_ros2",
        executable="imu_multi_node",
        name="im900_foot_multi_node",
        output="screen",
        parameters=[
            config,
            {
                "ports": [
                    LaunchConfiguration("left_port").perform(context),
                    LaunchConfiguration("right_port").perform(context),
                ]
            },
        ],
        remappings=IMU_REMAPPINGS,
    )
    fusion = Node(
        package="pico_bridge",
        executable="pico_foot_imu_fusion",
        name="pico_foot_imu_fusion",
        output="screen",
        parameters=[
            {
                "left_imu_topic": "/imu/left_feet",
                "right_imu_topic": "/imu/right_feet",
                "left_imu_ready_topic": "/imu/left_feet/ready",
                "right_imu_ready_topic": "/imu/right_feet/ready",
                "left_mount_quaternion": _quaternion_argument(context, "left_mount_quaternion"),
                "right_mount_quaternion": _quaternion_argument(context, "right_mount_quaternion"),
                "max_imu_age_sec": ParameterValue(
                    LaunchConfiguration("max_imu_age_sec"), value_type=float
                ),
                "calibration_samples": ParameterValue(
                    LaunchConfiguration("calibration_samples"), value_type=int
                ),
                "imu_reset_settle_sec": ParameterValue(
                    LaunchConfiguration("imu_reset_settle_sec"), value_type=float
                ),
                "auto_calibrate_on_world_reset": ParameterValue(
                    LaunchConfiguration("auto_calibrate_on_world_reset"), value_type=bool
                ),
                "left_zero_z_service": LaunchConfiguration("left_zero_z_service"),
                "right_zero_z_service": LaunchConfiguration("right_zero_z_service"),
            }
        ],
    )
    return [driver, fusion]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("left_port", default_value=DEFAULT_LEFT_PORT),
            DeclareLaunchArgument("right_port", default_value=DEFAULT_RIGHT_PORT),
            DeclareLaunchArgument("max_imu_age_sec", default_value="0.08"),
            DeclareLaunchArgument("calibration_samples", default_value="60"),
            DeclareLaunchArgument("imu_reset_settle_sec", default_value="1.0"),
            DeclareLaunchArgument("auto_calibrate_on_world_reset", default_value="true"),
            DeclareLaunchArgument("left_zero_z_service", default_value="/im900/left_foot/zero_z_axis"),
            DeclareLaunchArgument("right_zero_z_service", default_value="/im900/right_foot/zero_z_axis"),
            DeclareLaunchArgument("left_mount_quaternion", default_value=DEFAULT_MOUNT_QUATERNION),
            DeclareLaunchArgument("right_mount_quaternion", default_value=DEFAULT_MOUNT_QUATERNION),
            OpaqueFunction(function=_setup),
        ]
    )
