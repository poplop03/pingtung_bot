import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mega_bridge')
    default_params = os.path.join(pkg, 'config', 'mega_bridge.yaml')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')

    node = Node(
        package='mega_bridge',
        executable='mega_bridge_node',
        name='mega_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'port': LaunchConfiguration('port')},
        ],
    )

    return LaunchDescription([params_arg, port_arg, node])
