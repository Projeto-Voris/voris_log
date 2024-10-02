from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import date


def generate_launch_description():
    return LaunchDescription([

        # Node definition
        Node(
            package='voris_log',
            executable='stereo_tools',
            name='stereo_tools'
        )
    ])
