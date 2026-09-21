from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    config_path = PathJoinSubstitution([
        FindPackageShare('pico_recorder'),
        'config',
        'collect_config.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=config_path),
        Node(
            package='pico_recorder',
            executable='pico_recorder_node',
            name='pico_recorder_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path')
            }]
        ),
        # The keyboard needs a real terminal; launch does not emulate one.
        # Run separately: ros2 run pico_recorder pico_recorder_keyboard
    ])
