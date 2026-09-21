import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('fisheye_camera'),
        'config',
        'fisheye_cameras.yaml',
    )

    return LaunchDescription([
        Node(
            package='fisheye_camera',
            executable='fisheye_camera_node',
            name='fisheye_camera_node',
            parameters=[config_file],
            output='screen',
        ),
    ])
