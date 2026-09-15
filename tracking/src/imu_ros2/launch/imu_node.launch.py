from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    report_hz = LaunchConfiguration("report_hz")
    frame_id = LaunchConfiguration("frame_id")
    topic = LaunchConfiguration("topic")
    battery_topic = LaunchConfiguration("battery_topic")
    magnetic_topic = LaunchConfiguration("magnetic_topic")
    magnetic_magnitude_topic = LaunchConfiguration("magnetic_magnitude_topic")
    calibrate_service = LaunchConfiguration("calibrate_service")
    battery_query_period_ms = LaunchConfiguration("battery_query_period_ms")
    enable_compass = LaunchConfiguration("enable_compass")
    use_reliable_qos = LaunchConfiguration("use_reliable_qos")
    force_positive_w = LaunchConfiguration("force_positive_w")
    use_quaternion_continuity = LaunchConfiguration("use_quaternion_continuity")
    clear_ins_position = LaunchConfiguration("clear_ins_position")
    clear_world_axes = LaunchConfiguration("clear_world_axes")
    restore_world_axes = LaunchConfiguration("restore_world_axes")
    target_address = LaunchConfiguration("target_address")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyUSB0",
            description="Serial device path for the IMU.",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="460800",
            description="Serial baudrate.",
        ),
        DeclareLaunchArgument(
            "report_hz",
            default_value="200",
            description="IMU active report rate in Hz.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="imu_node",
            description="Frame ID used in published messages.",
        ),
        DeclareLaunchArgument(
            "topic",
            default_value="imuData_raw",
            description="IMU data topic name.",
        ),
        DeclareLaunchArgument(
            "battery_topic",
            default_value="imuBattery",
            description="Battery state topic name.",
        ),
        DeclareLaunchArgument(
            "magnetic_topic",
            default_value="imuMagneticField",
            description="Magnetic field topic name.",
        ),
        DeclareLaunchArgument(
            "magnetic_magnitude_topic",
            default_value="magnetic_field_magnitude",
            description="Magnetic field magnitude topic name.",
        ),
        DeclareLaunchArgument(
            "calibrate_service",
            default_value="calibrate_orientation",
            description="Orientation calibration service name.",
        ),
        DeclareLaunchArgument(
            "battery_query_period_ms",
            default_value="5000",
            description="Battery status query period in milliseconds.",
        ),
        DeclareLaunchArgument(
            "force_positive_w",
            default_value="true",
            description="Force quaternion w >= 0 when no prior frame is available.",
        ),
        DeclareLaunchArgument(
            "use_reliable_qos",
            default_value="true",
            description="Publish IMU data with reliable QoS instead of SensorDataQoS.",
        ),
        DeclareLaunchArgument(
            "use_quaternion_continuity",
            default_value="true",
            description="Flip quaternion sign to keep consecutive frames on the same hemisphere.",
        ),
        DeclareLaunchArgument(
            "enable_compass",
            default_value="true",
            description="Enable 9-axis compass fusion.",
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
        Node(
            package="imu_ros2",
            executable="imu_node",
            name="imu_node",
            output="screen",
            parameters=[{
                "port": port,
                "baudrate": ParameterValue(baudrate, value_type=int),
                "report_hz": ParameterValue(report_hz, value_type=int),
                "frame_id": frame_id,
                "topic": topic,
                "battery_topic": battery_topic,
                "magnetic_topic": magnetic_topic,
                "magnetic_magnitude_topic": magnetic_magnitude_topic,
                "calibrate_service": calibrate_service,
                "battery_query_period_ms": ParameterValue(
                    battery_query_period_ms,
                    value_type=int,
                ),
                "enable_compass": ParameterValue(enable_compass, value_type=bool),
                "use_reliable_qos": ParameterValue(use_reliable_qos, value_type=bool),
                "force_positive_w": ParameterValue(force_positive_w, value_type=bool),
                "use_quaternion_continuity": ParameterValue(
                    use_quaternion_continuity,
                    value_type=bool,
                ),
                "clear_ins_position": ParameterValue(clear_ins_position, value_type=bool),
                "clear_world_axes": ParameterValue(clear_world_axes, value_type=bool),
                "restore_world_axes": ParameterValue(restore_world_axes, value_type=bool),
                "target_address": ParameterValue(target_address, value_type=int),
            }],
        ),
    ])
