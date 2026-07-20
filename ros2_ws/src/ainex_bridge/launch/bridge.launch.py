from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('socket_path', default_value='/tmp/motiond.sock'),
        DeclareLaunchArgument('request_timeout', default_value='2.0'),
        DeclareLaunchArgument('state_poll_hz', default_value='5.0'),
        Node(
            package='ainex_bridge',
            executable='bridge',
            name='ainex_bridge',
            output='screen',
            parameters=[{
                'socket_path': LaunchConfiguration('socket_path'),
                'request_timeout': LaunchConfiguration('request_timeout'),
                'state_poll_hz': LaunchConfiguration('state_poll_hz'),
            }],
        ),
    ])
