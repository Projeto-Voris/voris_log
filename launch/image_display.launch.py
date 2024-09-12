from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'monitor_name',
            default_value='DP-3',
            description='Monitor output port'
        ),
        DeclareLaunchArgument(
            'noise_freq',
            default_value='10.0',
            description='Random image noise frequence'
        ),
        Node(
            package='voris_log',  # Replace with your package name
            executable='noise_image.py',  # Replace with your executable name
            name='noise_image',
            output='screen',
            parameters=[
                {'frequency': LaunchConfiguration('noise_freq')},
                {'monitor_name': LaunchConfiguration('monitor_name')}
            ],
            remappings=[
                # Add any topic remappings here if needed
            ]
        ),
    ])
