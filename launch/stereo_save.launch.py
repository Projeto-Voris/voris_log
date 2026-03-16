from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from datetime import date
import os

def generate_launch_description():
    # Get user environment variable
    user = os.getenv('USER', 'default_user')  # Fallback to 'default_user' if $USER is not set
    today_date = date.today().strftime("%Y%m%d")  # Get the current date
    os.makedirs('/home/{}/Pictures/image_logs/SM2'.format(user), exist_ok=True)
    os.makedirs('/home/{}/Pictures/image_logs/SM3'.format(user), exist_ok=True)
    os.makedirs('/home/{}/Pictures/image_logs/SM4'.format(user), exist_ok=True)
    os.makedirs('/home/{}/Pictures/image_logs/Passive'.format(user), exist_ok=True)

    return LaunchDescription([
        # Declare the launch argument for 'SM'
        DeclareLaunchArgument(
            'SM',
            default_value='Passive',
            description='This is the sm argument'
        ),

        # Declare the launch argument for the saving path
        DeclareLaunchArgument(
            'saving_path',
            default_value=PathJoinSubstitution([
                TextSubstitution(text='/home/{}/Pictures/image_logs'.format(user)),
                LaunchConfiguration('SM'),
                TextSubstitution(text='{}'.format(today_date)),
            ]),
            description='Path to save images'
        ),

        # Declare launch arguments to allow remapping of image topics
        DeclareLaunchArgument(
            'camera_1_image_topic',
            default_value=PathJoinSubstitution([
                TextSubstitution(text='/'),
                LaunchConfiguration('SM'),
                TextSubstitution(text='left/image_raw')]),

            description='Topic for camera 1 image stream'
        ),
        DeclareLaunchArgument(
            'camera_2_image_topic',
            default_value=PathJoinSubstitution([
                TextSubstitution(text='/'),
                LaunchConfiguration('SM'),
                TextSubstitution(text='right/image_raw')]),
            description='Topic for camera 2 image stream'
        ),

        # Node definition
        Node(
            package='voris_log',
            executable='stereo_save',
            namespace=LaunchConfiguration('SM'),
            name='stereo_save',
            output='screen',
            parameters=[{'saving_path': LaunchConfiguration('saving_path')}],
            remappings=[
                ('/camera_1/image_raw', LaunchConfiguration('camera_1_image_topic')),
                ('/camera_2/image_raw', LaunchConfiguration('camera_2_image_topic')),
                ('/save_images', PathJoinSubstitution([TextSubstitution(text='/'),
                                                       LaunchConfiguration('SM'),
                                                       TextSubstitution(text='save_image')]))
            ]
        )
    ])
