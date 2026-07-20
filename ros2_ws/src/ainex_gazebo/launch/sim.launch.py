"""AiNex Gazebo Classic simulation bringup (ROS2 Humble).

Starts: gzserver (+ gzclient unless headless:=true), robot_state_publisher,
spawn_entity, ros2_control spawners (joint_state_broadcaster +
joint_group_position_controller) and web_video_server (default port 8081).

Stream the fixed world camera at:
    http://<host>:8081/stream?topic=/sim_camera/image_raw
Snapshot:
    http://<host>:8081/snapshot?topic=/sim_camera/image_raw
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ainex_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    video_port = LaunchConfiguration('video_port')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run gzserver only (no gzclient GUI)')
    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_share, 'worlds', 'ainex_flat.world'),
        description='SDF world file')
    declare_video_port = DeclareLaunchArgument(
        'video_port', default_value='8081',
        description='HTTP port for web_video_server')

    # Gazebo Classic server / client
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'pause': 'false'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=UnlessCondition(headless),
    )

    # Robot description from xacro
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            os.path.join(pkg_share, 'urdf', 'ainex.xacro'),
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawn at z=0.25 like the ROS1 gazebo.launch (arm initial joint angles are
    # set via ros2_control initial_value in urdf/ros2_control.xacro since
    # spawn_entity.py has no -J equivalent).
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'ainex',
                   '-x', '0', '-y', '0', '-z', '0.25'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )

    joint_group_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_group_position_controller',
                   '--controller-manager', '/controller_manager'],
    )

    # Start controllers only after the robot is spawned (gazebo_ros2_control's
    # controller_manager only exists once the model is in the sim).
    spawn_then_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ))
    broadcaster_then_position = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_group_position_controller_spawner],
        ))

    # HTTP MJPEG streaming of /sim_camera/image_raw (and any other image topic)
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
        parameters=[{
            'port': ParameterValue(video_port, value_type=int),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        declare_headless,
        declare_world,
        declare_video_port,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        spawn_then_broadcaster,
        broadcaster_then_position,
        web_video_server,
    ])
