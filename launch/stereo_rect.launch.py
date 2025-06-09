from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
                'namespace',
                default_value='SM2',
                description='Namespace for the stereo cameras'    ) ,
        Node(
            package='voris_log',
            executable='image_rect',
            name='left_rectify',
            namespace=PathJoinSubstitution([LaunchConfiguration('namespace'), 'left']),
            remappings=[
                ('image_raw', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', '/left/image_rect'),
                ('camera_info_rect', '/left/camera_info')
            ]
        ),
        Node(
            package='voris_log',
            executable='image_rect',
            name='right_rectify',
            namespace=PathJoinSubstitution([LaunchConfiguration('namespace'), 'right']),
            remappings=[
                ('image_raw', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', '/right/image_rect'),
                ('camera_info_rect', '/right/camera_info')
            ]
        )
    ])