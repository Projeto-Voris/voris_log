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
            executable='stereo_rectify',
            name='stereo_rectify',
            namespace=LaunchConfiguration('namespace'),
            remappings=[
                ('left/image_raw', 'left/image_raw'),
                ('left/camera_info', 'left/camera_info'),
                ('right/image_raw', 'right/image_raw'),
                ('right/camera_info', 'right/camera_info'),
                ('left/image_rect', '/left/image_rect'),
                ('right/image_rect', '/right/image_rect'),
                ('left/camera_info_rect', '/left/camera_info'),
                ('right/camera_info_rect', '/right/camera_info'),
            ]
        ),
       
    ])