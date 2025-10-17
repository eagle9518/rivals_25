import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'rivals'

    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file, ' use_ros2_control:=true', ' sim_mode:=false'])}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    camera_one = IncludeLaunchDescription(
                     PathJoinSubstitution([
                         FindPackageShare(package_name),
                         'launch',
                         'camera.launch.py'
                     ]),
    )

    publish_poses = IncludeLaunchDescription(
                        PathJoinSubstitution([
                            FindPackageShare(package_name),
                            'launch',
                            'poses.launch.py'
                        ]),
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out', '/mecanum_controller/reference_unstamped')]
    )

    controller_params = os.path.join(pkg_path, 'config', 'control.yaml')
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # TODO: Yeah probably should make a separate laucnh file for this
    shooter_servo_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["shooter_position_controller", "--controller-manager", "/controller_manager"]
    )
    shooter_wheel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["shooter_velocity_controller", "--controller-manager", "/controller_manager"]
    )
    intake_servo_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["intake_position_controller", "--controller-manager", "/controller_manager"]
    )
    intake_wheel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["intake_velocity_controller", "--controller-manager", "/controller_manager"]
    )
    stick_servo_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["stick_position_controller", "--controller-manager", "/controller_manager"]
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"]
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"]
    )

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[shooter_servo_spawner, shooter_wheel_spawner, 
                      intake_servo_spawner, intake_wheel_spawner, 
                      stick_servo_spawner,
                      diff_drive_spawner, joint_broad_spawner
            ]
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        camera_one,
        publish_poses,
        twist_mux,
        controller_manager_node,
        delayed_spawners
    ])
