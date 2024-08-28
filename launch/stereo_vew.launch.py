from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import date


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_1_image_topic',
            default_value='/SM3/left/image_raw',
            description='Topic for camera 1 image stream'
        ),
        DeclareLaunchArgument(
            'camera_2_image_topic',
            default_value='/SM3/right/image_raw',
            description='Topic for camera 2 image stream'
        ),

        # Node definition
        Node(
            package='voris_log',
            executable='stereo_view',
            name='stereo_view',
            output='screen',
            remappings=[
                ('/camera_1/image_raw', LaunchConfiguration('camera_1_image_topic')),
                ('/camera_2/image_raw', LaunchConfiguration('camera_2_image_topic'))
            ]
        )
    ])
