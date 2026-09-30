"""Start the whole robot for fruit picking, plus the web UI.

    bringup.launch.py  ── IMU, wheel_control, mega_bridge (all hardware)
    vision.launch.py   ── camera + vision node in fruit_color mode
    pick_fruit_node    ── state machine + web UI at http://<robot-ip>:<web_port>/

Keep the robot still for the first few seconds while wheel_control
calibrates the gyro.

    ros2 launch pick_fruit pick_fruit.launch.py
    ros2 launch pick_fruit pick_fruit.launch.py port:=/dev/ttyACM1 video_device:=/dev/video0
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_launch = os.path.join(
        get_package_share_directory('pingtung_bot_bringup'), 'launch', 'bringup.launch.py')
    vision_launch = os.path.join(
        get_package_share_directory('pingtung_vision'), 'launch', 'vision.launch.py')
    default_params = os.path.join(
        get_package_share_directory('pick_fruit'), 'config', 'pick_fruit.yaml')
    wheel_control_params = os.path.join(
        get_package_share_directory('wheel_control'), 'config', 'wheel_control.yaml')

    # pick_fruit sends linear.x = pwm / k_lin, so it must use wheel_control's k_lin
    with open(wheel_control_params) as f:
        k_lin = float(yaml.safe_load(f)['/wheel_control']['ros__parameters']['k_lin'])

    args = [
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='USB serial port of the Arduino Mega'),
        DeclareLaunchArgument('video_device', default_value='/dev/video4'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('pick_params_file', default_value=default_params),
    ]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={'port': LaunchConfiguration('port'), 'base': 'true'}.items(),
    )

    # pick_fruit serves the camera itself, so vision's own web view stays off,
    # but the annotated debug image is still needed for it.
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_launch),
        launch_arguments={
            'vision_mode': 'fruit_color',
            'video_device': LaunchConfiguration('video_device'),
            'web_view': 'false',
            'publish_debug_image': 'true',
        }.items(),
    )

    pick_fruit = Node(
        package='pick_fruit',
        executable='pick_fruit_node',
        name='pick_fruit',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('pick_params_file'),
            {'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
             'k_lin': k_lin},
        ],
    )

    return LaunchDescription(args + [hardware, vision, pick_fruit])
