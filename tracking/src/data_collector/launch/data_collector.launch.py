from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    config_path = PathJoinSubstitution([
        FindPackageShare('data_collector'),
        'config',
        'collect_config.yaml'
    ])

    return LaunchDescription([
        # Data collector node
        Node(
            package='data_collector',
            executable='data_collector_node',
            name='data_collector_node',
            output='screen',
            parameters=[{
                'config_path': config_path
            }]
        ),
        # keyboard_controller needs a real terminal (stdin),
        # run it separately: ros2 run data_collector keyboard_controller
    ])
