import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_path = LaunchConfiguration('pkg_path')
    pkg_path = DeclareLaunchArgument(
                    'pkg_path',
                    default_value="rivals"
               )
    april_settings = os.path.join(pkg_path, 'config', 'april_settings.yaml')
    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="apriltag_container",
        namespace="",
        composable_node_descriptions=[
            ComposableNode(
                package="v4l2_camera",
                plugin="v4l2_camera::V4L2Camera",
                name="camera1",
                namespace="cam1",
                parameters=[{
                    "video_device": "/dev/video0",
                    "image_size": [640, 480],
                    "use_intra_process_comms": True,
                    "time_per_frame": [1, 30] #30 fps
                }],
            ),
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag1",
                namespace="cam1",
                parameters=[
                    {"use_intra_process_comms": True},
                    april_settings
                ],
                remappings=[
                    ("image_rect", "image_raw"),
                    ("camera_info", "camera_info")
                ]
            ),
        ],
        output="screen"
    )

    return LaunchDescription([container])
