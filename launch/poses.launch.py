import os, yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'rivals'
    pkg_path = get_package_share_directory(package_name)

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
                executable='static_transform_publisher',
                name=f'tag{tag}_tfpub',
                arguments=[
                    str(x), str(y), str(z),
                    str(yaw), str(pitch), str(roll),
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
                executable='static_transform_publisher',
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
