import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Lightweight launch for odometry + SLAM/relocalization only.

    Starts ONLY the main driver node (point clouds, odom/imu, and the
    map->odom relocalization TF). The post_process / reprojector /
    pointcloud_to_depth nodes are intentionally NOT launched, so there are no
    'process finished' messages from disabled nodes and minimal CPU usage.
    """
    pkg_share = get_package_share_directory('odin_ros_driver_rev1')
    default_rviz = os.path.join(pkg_share, 'rviz', 'odin_slam.rviz')

    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    image_fps = LaunchConfiguration('image_fps')
    image_format = LaunchConfiguration('image_format')
    topic_prefix = LaunchConfiguration('topic_prefix')

    driver_params = {
        'operating_mode': 'normal',
        'image_width': image_width,
        'image_height': image_height,
        'image_fps': image_fps,
        'image_format': image_format,
        'topic_prefix': topic_prefix,
        # Use a dedicated config so this launch never interferes with the
        # full driver.launch.py (which keeps using control_command.yaml).
        'channel_config_file': 'control_command_slam.yaml',
    }

    nodes = [
        Node(
            package='odin_ros_driver_rev1',
            executable='odin_ros_driver_node',
            output='screen',
            parameters=[driver_params],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='odin_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(start_rviz),
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('image_width', default_value='0'),
        DeclareLaunchArgument('image_height', default_value='0'),
        DeclareLaunchArgument('image_fps', default_value='0'),
        DeclareLaunchArgument('image_format', default_value='mjpeg'),
        DeclareLaunchArgument('topic_prefix', default_value='manifold'),
        *nodes,
    ])
