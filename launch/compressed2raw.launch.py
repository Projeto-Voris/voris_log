import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments to specify YAML file paths for left and right camera
        DeclareLaunchArgument('image_output', default_value='image_raw', description='Image topic name'),
        DeclareLaunchArgument('namespace', default_value='Passive', description='namespace'),

        # Node configuration
        Node(
            package='voris_log',  # replace with your actual package name
            executable='compressed2raw',
            name='compressed2raw_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            remappings=[
                ('image_raw', LaunchConfiguration('image_output')),
                ('image/compressed', '/my_camera/image_raw/compressed')  # Remap to your actual compressed image topic
            ]
        ),
    ])
