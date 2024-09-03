from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lc = LaunchContext()
    joy_type = EnvironmentVariable('CPR_JOY_TYPE', default_value='logitech')

    filepath_config_joy = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', ('teleop_' + joy_type.perform(lc) + '.yaml')]
    )

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
    )

    ld = LaunchDescription()
    ld.add_action(node_joy)
    ld.add_action(joy2bot)
    return ld
