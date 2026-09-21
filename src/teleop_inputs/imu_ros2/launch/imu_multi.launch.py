from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ports = [
        "/dev/ttyACM1",
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
        "/dev/ttyUSB2",
        "/dev/ttyUSB3",
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Serial baudrate shared by all IMUs.",
        ),
        DeclareLaunchArgument(
            "report_hz",
            default_value="110",
            description="IMU active report rate in Hz.",
        ),
        DeclareLaunchArgument(
            "battery_query_period_ms",
            default_value="5000",
            description="Battery status query period in milliseconds.",
        ),
        DeclareLaunchArgument(
            "enable_compass",
            default_value="true",
            description="Enable 9-axis compass fusion.",
        ),
        DeclareLaunchArgument(
            "use_reliable_qos",
            default_value="false",
            description="Publish IMU data with reliable QoS instead of SensorDataQoS.",
        ),
        DeclareLaunchArgument(
            "use_device_timestamp",
            default_value="true",
            description="Use IMU device_time_ms for message timestamps.",
        ),
        DeclareLaunchArgument(
            "coalesce_frames_per_poll",
            default_value="true",
            description="Publish only the newest IMU frame gathered in each poll cycle.",
        ),
        DeclareLaunchArgument(
            "align_quaternion_hemisphere",
            default_value="true",
            description="Flip quaternion sign to keep consecutive frames on the same hemisphere.",
        ),
        DeclareLaunchArgument(
            "force_positive_w",
            default_value="true",
            description="Force quaternion w >= 0 when no prior frame is available.",
        ),
        DeclareLaunchArgument(
            "use_quaternion_continuity",
            default_value=LaunchConfiguration("align_quaternion_hemisphere"),
            description=(
                "Legacy alias for align_quaternion_hemisphere; defaults to the same value "
                "unless explicitly overridden."
            ),
        ),
        DeclareLaunchArgument(
            "clear_ins_position",
            default_value="true",
            description="Clear INS position on startup.",
        ),
        DeclareLaunchArgument(
            "clear_world_axes",
            default_value="false",
            description="Clear world axes on startup.",
        ),
        DeclareLaunchArgument(
            "restore_world_axes",
            default_value="false",
            description="Restore default world axes on startup.",
        ),
        DeclareLaunchArgument(
            "target_address",
            default_value="255",
            description="IMU target address. 255 is broadcast.",
        ),
        DeclareLaunchArgument(
            "name_prefix",
            default_value="imu",
            description="Channel name prefix used to generate imu0, imu1, ...",
        ),
        DeclareLaunchArgument(
            "magnetic_topic",
            default_value="imuMagneticField",
            description="Per-channel magnetic field topic suffix.",
        ),
        DeclareLaunchArgument(
            "magnetic_magnitude_topic",
            default_value="magnetic_field_magnitude",
            description="Per-channel magnetic field magnitude topic suffix.",
        ),
        Node(
            package="imu_ros2",
            executable="imu_multi_node",
            name="imu_multi_node",
            output="screen",
            parameters=[{
                "ports": ports,
                "baudrate": ParameterValue(LaunchConfiguration("baudrate"), value_type=int),
                "report_hz": ParameterValue(LaunchConfiguration("report_hz"), value_type=int),
                "battery_query_period_ms": ParameterValue(
                    LaunchConfiguration("battery_query_period_ms"),
                    value_type=int,
                ),
                "enable_compass": ParameterValue(
                    LaunchConfiguration("enable_compass"),
                    value_type=bool,
                ),
                "use_reliable_qos": ParameterValue(
                    LaunchConfiguration("use_reliable_qos"),
                    value_type=bool,
                ),
                "use_device_timestamp": ParameterValue(
                    LaunchConfiguration("use_device_timestamp"),
                    value_type=bool,
                ),
                "coalesce_frames_per_poll": ParameterValue(
                    LaunchConfiguration("coalesce_frames_per_poll"),
                    value_type=bool,
                ),
                "align_quaternion_hemisphere": ParameterValue(
                    LaunchConfiguration("align_quaternion_hemisphere"),
                    value_type=bool,
                ),
                "force_positive_w": ParameterValue(
                    LaunchConfiguration("force_positive_w"),
                    value_type=bool,
                ),
                "use_quaternion_continuity": ParameterValue(
                    LaunchConfiguration("use_quaternion_continuity"),
                    value_type=bool,
                ),
                "clear_ins_position": ParameterValue(
                    LaunchConfiguration("clear_ins_position"),
                    value_type=bool,
                ),
                "clear_world_axes": ParameterValue(
                    LaunchConfiguration("clear_world_axes"),
                    value_type=bool,
                ),
                "restore_world_axes": ParameterValue(
                    LaunchConfiguration("restore_world_axes"),
                    value_type=bool,
                ),
                "target_address": ParameterValue(
                    LaunchConfiguration("target_address"),
                    value_type=int,
                ),
                "name_prefix": LaunchConfiguration("name_prefix"),
                "magnetic_topic": LaunchConfiguration("magnetic_topic"),
                "magnetic_magnitude_topic": LaunchConfiguration(
                    "magnetic_magnitude_topic"
                ),
            }],
        ),
    ])
