from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import date
import os


def generate_launch_description():
    user = os.getenv('USER')
    sm = 'SM3'

    return LaunchDescription([
        # Declare launch arguments to allow remapping of image topics
        DeclareLaunchArgument(
            'saving_path',
            default_value='/home/{}/Pictures/SM3-{}'.format( user, date.today().strftime("%Y%m%d")),
            description='Path to save images'
        ),
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
        DeclareLaunchArgument(
            'pattern_topic',
            default_value='/pattern_change',
            description='Service name for projector node'
        ),
        DeclareLaunchArgument(
            'use_pattern',
            default_value='True',
            description='Node of projection will be used?'
        ),

        # Node definition
        Node(
            package='voris_log',
            executable='stereo_save',
            name='stereo_save',
            output='screen',
            parameters=[
                {'saving_path': LaunchConfiguration('saving_path')},
                {'pattern_topic': LaunchConfiguration('pattern_topic')},
                {'use_pattern': LaunchConfiguration('use_pattern')}
            ],
            remappings=[
                ('/camera_1/image_raw', LaunchConfiguration('camera_1_image_topic')),
                ('/camera_2/image_raw', LaunchConfiguration('camera_2_image_topic'))
            ]
        )
    ])
