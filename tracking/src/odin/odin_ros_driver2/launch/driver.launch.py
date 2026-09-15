import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('odin_ros_driver_rev1')
    default_rviz = os.path.join(pkg_share, 'rviz', 'driver_view_ros2.rviz')

    # Ports / device SN are auto-handled by SDK discovery, no need to configure them
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    # Image resolution: 0 = use first capability from device
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    image_fps = LaunchConfiguration('image_fps')
    image_format = LaunchConfiguration('image_format')
    # Topic prefix base (default: "manifold")
    # Final topic format: /{topic_prefix}/{model}/device{index}/
    topic_prefix = LaunchConfiguration('topic_prefix')

    driver_params = {
        'operating_mode': 'normal',
        'image_width': image_width,
        'image_height': image_height,
        'image_fps': image_fps,
        'image_format': image_format,
        'topic_prefix': topic_prefix,
        'channel_config_file': 'control_command.yaml',
    }

    nodes = [
        Node(
            package='odin_ros_driver_rev1',
            executable='odin_ros_driver_node',
            output='screen',
            parameters=[driver_params],
        ),
        # Node(
        #     package='image_transport',
        #     executable='republish',
        #     name='odin_image_republish',
        #     arguments=[
        #         'compressed',
        #         'raw',
        #         '--ros-args',
        #         '--remap', 'in/compressed:=/odin/image/compressed',
        #         '--remap', 'out:=/odin/image_raw',
        #     ],
        #     output='screen',
        #     condition=IfCondition(start_rviz),
        # ),
        # --- NEW: cloudrender with YAML under src/ ---
        Node(
            package='odin_ros_driver_rev1',
            executable='post_process_node',
            name='postprocess',
            output='screen',
            parameters=[{'topic_prefix': topic_prefix}],
        ),
        Node(
            package='odin_ros_driver_rev1',
            executable='pointcloud_reprojector_node',
            name='pointcloud_reprojector',
            output='screen',
            parameters=[{
                'mode': 'raw',
                'min_depth': 0.1,
                'max_depth': 50.0,
                'point_size': 2,
                'topic_prefix': topic_prefix,
            }],
        ),
        # PointCloud -> depth map verification node.
        # Subscribes to the synchronized cloud_raw + camera0/undistort and
        # publishes /{prefix}/{model}/device{N}/pointcloud2depth/{depth,depth_color,depth_overlay}.
        Node(
            package='odin_ros_driver_rev1',
            executable='pointcloud_to_depth_node',
            name='pointcloud_to_depth',
            output='screen',
            parameters=[{
                'topic_prefix': topic_prefix,
                # Projection canvas downscale factor (see depth-pipeline-handoff doc).
                'scale': 7.0,
                # Valid depth range (meters).
                'z_min': 0.1,
                'z_max': 50.0,
                # 3x3 dilation when writing projected pixels (fills small holes).
                'dilate_radius': 1,
                # No z-buffer: overwrite-only on empty pixels, matches doc spec.
                'use_z_buffer': False,
                # Sobel edge suppression threshold (m, on full-res depth).
                'edge_grad_threshold': 0.75,
                # Fixed jet-colormap range (m); set vis_depth_min < 0 to
                # auto-normalize per-frame instead.
                'vis_depth_min': 0.1,
                'vis_depth_max': 5.0,
            }],
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
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('image_width', default_value='0'),
        DeclareLaunchArgument('image_height', default_value='0'),
        DeclareLaunchArgument('image_fps', default_value='0'),
        DeclareLaunchArgument('image_format', default_value='mjpeg'),
        DeclareLaunchArgument('topic_prefix', default_value='manifold'),
        *nodes,
    ])
