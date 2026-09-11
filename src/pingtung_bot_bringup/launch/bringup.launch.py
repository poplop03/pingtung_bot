import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ==========================================
    # 1. Resolve Parameter File Paths
    # ==========================================
    # Resolving path for bno055 parameter file
    bno055_params = os.path.join(
        get_package_share_directory('bno055'),
        'params',
        'bno055_params_i2c.yaml'
    )

    # Resolving path for wheel_control parameter file
    wheel_control_params = os.path.join(
        get_package_share_directory('wheel_control'),
        'config',
        'wheel_control.yaml'
    )

    # ==========================================
    # 2. Define Node Actions
    # ==========================================
    # Command 1: ros2 run bno055 bno055 ...
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055_node',      # Explicitly named for cleaner ros2 node list output
        output='screen',
        parameters=[bno055_params]
    )

    # Command 2: ros2 run wheel_control wheel_control_node ...
    wheel_control_node = Node(
        package='wheel_control',
        executable='wheel_control_node',
        name='wheel_control_node',
        output='screen',
        parameters=[wheel_control_params]
    )

    # ==========================================
    # 3. Return Launch Description
    # ==========================================
    return LaunchDescription([
        bno055_node,
        wheel_control_node
    ])
