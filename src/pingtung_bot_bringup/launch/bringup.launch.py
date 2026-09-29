"""Start the whole robot: differential base + gantry, all through the Arduino Mega.

    bno055 ──/bno055/imu──► wheel_control ──/mega/wheel_pwm──┐
    /mega/cmd_vel ─────────►  (base)                         ▼
    /mega/step ──────────────────────────────────────► mega_bridge ◄──USB──► Mega
    /mega/gripper ───────────────────────────────────►  (gantry +           (wheels, steppers,
    /mega/status ◄───────────────────────────────────    serial owner)       servo)

mega_bridge is the only process that opens the serial port, so it always runs.
`base:=false` leaves out the IMU and wheel_control, e.g. to work on the gantry
with the robot on a bench. `gantry_test:=true` also runs mega_bridge's
gantry_test node, which moves each gantry axis out and back and cycles the
gripper, then exits.

    ros2 launch pingtung_bot_bringup bringup.launch.py
    ros2 launch pingtung_bot_bringup bringup.launch.py port:=/dev/ttyACM1 base:=false
    ros2 launch pingtung_bot_bringup bringup.launch.py base:=false gantry_test:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bno055_params = os.path.join(
        get_package_share_directory('bno055'), 'config', 'bno055_params_i2c.yaml')
    wheel_control_params = os.path.join(
        get_package_share_directory('wheel_control'), 'config', 'wheel_control.yaml')
    mega_bridge_params = os.path.join(
        get_package_share_directory('mega_bridge'), 'config', 'mega_bridge.yaml')

    port = LaunchConfiguration('port')
    base = LaunchConfiguration('base')
    gantry_test = LaunchConfiguration('gantry_test')

    args = [
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='USB serial port of the Arduino Mega'),
        DeclareLaunchArgument('base', default_value='true',
                              description='start the IMU and wheel_control'),
        DeclareLaunchArgument('gantry_test', default_value='false',
                              description='run the gantry/gripper test sequence once'),
    ]

    # ---- gantry + serial link: steppers, gripper, and wheel PWM passthrough ----
    mega_bridge = Node(
        package='mega_bridge',
        executable='mega_bridge_node',
        name='mega_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[mega_bridge_params, {'port': port}],
    )

    # ---- differential base: IMU + heading/yaw-rate loop ----
    base_nodes = GroupAction(
        condition=IfCondition(base),
        actions=[
            Node(
                package='bno055',
                executable='bno055',
                name='bno055',   # must match the top-level key in bno055_params_i2c.yaml
                output='screen',
                parameters=[bno055_params],
            ),
            Node(
                package='wheel_control',
                executable='wheel_control_node',
                name='wheel_control',
                output='screen',
                emulate_tty=True,
                # 'topic': publish /mega/wheel_pwm to mega_bridge instead of
                # opening the serial port itself
                parameters=[wheel_control_params, {'output': 'topic'}],
                remappings=[('cmd_vel', '/mega/cmd_vel')],
            ),
        ],
    )

    # ---- optional: gantry + gripper self-test, exits when finished ----
    gantry_test_node = Node(
        condition=IfCondition(gantry_test),
        package='mega_bridge',
        executable='gantry_test',
        name='gantry_test',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription(args + [mega_bridge, base_nodes, gantry_test_node])
