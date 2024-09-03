import os

from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'joycon'

    lc = LaunchContext()
    joy_type = EnvironmentVariable('CPR_JOY_TYPE', default_value='logitech')

    filepath_config_joy = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', ('teleop_' + joy_type.perform(lc) + '.yaml')]
    )

    jackal_config = os.path.join(get_package_share_directory(package_name), 'config', 'jackal_config.yaml')

    node_joy = Node(
        namespace='joy_teleop',
        package='joy',
        executable='joy_node',
        output='screen',
        name='joy_node',
        parameters=[filepath_config_joy]
    )

    joy2bot = Node(
        namespace='joy_teleop',
        package='joycon',
        executable='joy2bot',
        output='screen',
        name='joy2bot',
        parameters=[jackal_config]
    )

    micro_ros_bridge = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0'],
    )


    ld = LaunchDescription()
    ld.add_action(node_joy)
    ld.add_action(joy2bot)
    return ld
