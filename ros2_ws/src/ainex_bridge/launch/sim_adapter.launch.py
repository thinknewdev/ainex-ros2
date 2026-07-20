from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# joint_order MUST be exactly the joints list (same names, same order) of the
# ainex_gazebo joint_group_position_controller this node feeds. This default is
# the proposed convention (motiond walking joints in vendor joint_index order,
# then head); override at launch if the gazebo controller config differs:
#   ros2 launch ainex_bridge sim_adapter.launch.py \
#       joint_order:='[r_hip_yaw, ..., head_tilt]'
DEFAULT_JOINT_ORDER = "[r_hip_yaw, r_hip_roll, r_hip_pitch, r_knee, r_ank_pitch, r_ank_roll, l_hip_yaw, l_hip_roll, l_hip_pitch, l_knee, l_ank_pitch, l_ank_roll, r_sho_pitch, l_sho_pitch, l_sho_roll, r_sho_roll, l_el_pitch, r_el_pitch, l_el_yaw, r_el_yaw, l_gripper, r_gripper, head_pan, head_tilt]"

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('udp_port', default_value='9910'),
        DeclareLaunchArgument('udp_bind', default_value='127.0.0.1'),
        DeclareLaunchArgument('publish_rate', default_value='50.0'),
        DeclareLaunchArgument('staleness_timeout', default_value='1.0'),
        DeclareLaunchArgument('joint_order', default_value=DEFAULT_JOINT_ORDER),
        Node(
            package='ainex_bridge',
            executable='sim_adapter',
            name='sim_adapter',
            output='screen',
            parameters=[{
                'udp_port': ParameterValue(
                    LaunchConfiguration('udp_port'), value_type=int),
                'udp_bind': LaunchConfiguration('udp_bind'),
                'publish_rate': ParameterValue(
                    LaunchConfiguration('publish_rate'), value_type=float),
                'staleness_timeout': ParameterValue(
                    LaunchConfiguration('staleness_timeout'), value_type=float),
                # value_type=None -> yaml-evaluated, so the "[a, b, c]" string
                # becomes a string array parameter.
                'joint_order': ParameterValue(
                    LaunchConfiguration('joint_order'), value_type=None),
            }],
        ),
    ])
