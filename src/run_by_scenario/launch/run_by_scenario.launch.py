"""Start the robot base plus the record/playback web UI.

    bringup.launch.py    ── IMU, wheel_control, mega_bridge (all hardware)
    run_by_scenario_node ── web UI at http://<robot-ip>:<web_port>/

Keep the robot still for the first few seconds while wheel_control
calibrates the gyro. Do not run it together with pick_fruit: both drive
/mega/cmd_vel and both serve port 8080 by default.

    ros2 launch run_by_scenario run_by_scenario.launch.py
    ros2 launch run_by_scenario run_by_scenario.launch.py port:=/dev/ttyACM1 web_port:=8081
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
    default_params = os.path.join(
        get_package_share_directory('run_by_scenario'), 'config', 'run_by_scenario.yaml')
    wheel_control_params = os.path.join(
        get_package_share_directory('wheel_control'), 'config', 'wheel_control.yaml')

    # the node sends linear.x = pwm / k_lin and angular.z = pwm / k_ff, so it must
    # use wheel_control's k_lin and k_ff
    with open(wheel_control_params) as f:
        wheel_control = yaml.safe_load(f)['/wheel_control']['ros__parameters']
    k_lin, k_ff = float(wheel_control['k_lin']), float(wheel_control['k_ff'])

    args = [
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='USB serial port of the Arduino Mega'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('scenario_params_file', default_value=default_params),
    ]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={'port': LaunchConfiguration('port'), 'base': 'true'}.items(),
    )

    node = Node(
        package='run_by_scenario',
        executable='run_by_scenario_node',
        name='run_by_scenario',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('scenario_params_file'),
            {'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
             'k_lin': k_lin, 'k_ff': k_ff},
        ],
    )

    return LaunchDescription(args + [hardware, node])
