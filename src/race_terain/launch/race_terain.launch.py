"""Robot base + RealSense depth + race_terain (web UI at http://<robot-ip>:<web_port>/).

    bringup.launch.py   ── IMU, wheel_control, mega_bridge
    realsense2_camera   ── D415 depth only, 424x240 @ 30 fps
    race_terain_node    ── centre-line following, /mega/cmd_vel

Keep the robot still for the first few seconds (wheel_control calibrates the
gyro), and again for ~1 s after START. Do not run it together with
pick_fruit / lane_runner / pingtung_vision: same camera, same port 8080.

    ros2 launch race_terain race_terain.launch.py
    ros2 launch race_terain race_terain.launch.py dry_run:=true     # watch only, no driving
    ros2 launch race_terain race_terain.launch.py hardware:=false   # camera + node only
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory
    bringup_launch = os.path.join(share('pingtung_bot_bringup'), 'launch', 'bringup.launch.py')
    realsense_launch = os.path.join(share('realsense2_camera'), 'launch', 'rs_launch.py')
    default_params = os.path.join(share('race_terain'), 'config', 'race_terain.yaml')
    with open(os.path.join(share('wheel_control'), 'config', 'wheel_control.yaml')) as f:
        wheel_control = yaml.safe_load(f)['/wheel_control']['ros__parameters']

    args = [
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='USB serial port of the Arduino Mega'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true = never drive, only show what it sees'),
        DeclareLaunchArgument('hardware', default_value='true',
                              description='false = do not start IMU / wheel_control / mega_bridge'),
        DeclareLaunchArgument('params_file', default_value=default_params),
    ]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        condition=IfCondition(LaunchConfiguration('hardware')),
        launch_arguments={'port': LaunchConfiguration('port'), 'base': 'true'}.items(),
    )

    # the same camera setup as lane_runner: depth only, the node makes the point cloud itself
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'enable_color': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_depth': 'true',
            'depth_module.depth_profile': '424x240x30',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
        }.items(),
    )

    node = Node(
        package='race_terain',
        executable='race_terain_node',
        name='race_terain',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
             'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
             'k_lin': float(wheel_control['k_lin']),
             'k_ff': float(wheel_control['k_ff']),
             'imu_yaw_sign': float(wheel_control['imu_yaw_sign'])},
        ],
    )

    return LaunchDescription(args + [hardware, camera, node])
