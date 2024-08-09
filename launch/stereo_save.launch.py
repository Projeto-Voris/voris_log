from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import date


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments to allow remapping of image topics
        # Declare launch argument for the saving path
        DeclareLaunchArgument(
            'saving_path',
            default_value='/home/daniel/Pictures/SM3-{}'.format(date.today().strftime("%Y%m%d")),
            description='Path to save images'
        ),
        DeclareLaunchArgument(
            'visualize',
            default_value='true',
            description='Path to save images'
        ),
        DeclareLaunchArgument(
            'camera_1_image_topic',
            default_value='/left/image_raw',
            description='Topic for camera 1 image stream'
        ),
        DeclareLaunchArgument(
            'camera_2_image_topic',
            default_value='/right/image_raw',
            description='Topic for camera 2 image stream'
        ),

        # Node definition
        Node(
            package='voris_log',
            executable='stereo_save',
            name='stereo_save',
            output='screen',
            parameters=[{'saving_path': LaunchConfiguration('saving_path'), 'visualize': LaunchConfiguration('visualize')}],
            remappings=[
                ('/camera_1/image_raw', LaunchConfiguration('camera_1_image_topic')),
                ('/camera_2/image_raw', LaunchConfiguration('camera_2_image_topic'))
            ]
        )
    ])
