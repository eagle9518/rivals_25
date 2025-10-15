import os, yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_path = LaunchConfiguration('pkg_path')
    pkg_path = DeclareLaunchArgument(
                    'pkg_path',
                    default_value='rivals'
    )
    tag_pose = os.path.join(pkg_path, 'config', 'tag_pose.yaml')
    target_pose = os.path.join(pkg_path, 'config', 'target_pose.yaml')

    with open(tag_pose, 'r') as f:
        tags = yaml.safe_load(f)['tags']

    with open(target_pose, 'r') as f:
        targets = yaml.safe_load(f)['targets']

    nodes = []
    for tag in tags:
        x, y, z = tags[tag]['xyz']
        roll, pitch, yaw = tags[tag]['rpy']

        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transformpublisher',
                name=f'tag{tag}_tfpub',
                arguments=[
                    str(x), str(y), str(z),
                    str(roll), str(pitch), str(yaw),
                    'world', f'tag{tag}'
                ],
                output='screen'
            )
        )

    for target in targets:
        x, y, z = targets[target]['xyz']
        roll, pitch, yaw = targets[target]['rpy']

        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transformpublisher',
                name=f'{target}_tfpub',
                arguments=[
                    str(x), str(y), str(z),
                    str(roll), str(pitch), str(yaw),
                    'world', target
                ],
                output='screen'
            )
        )
    return LaunchDescription(nodes)
