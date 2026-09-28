import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('pingtung_vision')
    default_params = os.path.join(package_share, 'config', 'vision.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params
    )
    mode_arg = DeclareLaunchArgument('vision_mode', default_value='off')
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera/image_raw'
    )
    start_camera_arg = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='Start the v4l2_camera node in this launch file',
    )
    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video6'
    )
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('start_camera')),
        parameters=[{
            'video_device': ParameterValue(
                LaunchConfiguration('video_device'), value_type=str
            ),
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'output_encoding': 'bgr8',
            'camera_frame_id': 'camera_link',
        }],
        remappings=[
            ('image_raw', LaunchConfiguration('camera_topic')),
        ],
    )
    node = Node(
        package='pingtung_vision',
        executable='vision_node',
        name='vision',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'algorithm': ParameterValue(
                    LaunchConfiguration('vision_mode'), value_type=str
                ),
                'camera_topic': ParameterValue(
                    LaunchConfiguration('camera_topic'), value_type=str
                ),
            },
        ],
    )
    return LaunchDescription(
        [
            params_arg,
            mode_arg,
            camera_topic_arg,
            start_camera_arg,
            video_device_arg,
            camera_node,
            node,
        ]
    )
