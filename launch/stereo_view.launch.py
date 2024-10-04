from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare the launch argument for 'SM'
        DeclareLaunchArgument(
            'SM',
            default_value='SM3',
            description='This is the sm argument'
        ),

        # Declare the launch arguments for the camera topics
        DeclareLaunchArgument(
            'camera_1_image_topic',
            default_value=PathJoinSubstitution([
                TextSubstitution(text='/'),
                LaunchConfiguration('SM'),
                TextSubstitution(text='left/image_raw')
            ]),
            description='Topic for camera 1 image stream'
        ),
        DeclareLaunchArgument(
            'camera_2_image_topic',
            default_value=PathJoinSubstitution([
                TextSubstitution(text='/'),
                LaunchConfiguration('SM'),
                TextSubstitution(text='right/image_raw')
            ]),
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
