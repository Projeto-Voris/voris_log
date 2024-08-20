from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_view',
            executable='stereo_view',
            name='stereo_view',
            remappings=[
                ('/camera_1/image_raw','sm3_left/image_raw'),
                ('/camera_2/image_raw', 'sm3_right/image_raw')],
            parameters=[{'_approximate_sync': True}]
        ),
    ])
