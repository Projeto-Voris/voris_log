from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import date
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg


def generate_launch_description():
    return LaunchDescription([
        LaunchArg( 'disparity_image', default_value=['disparity/image']),
        LaunchArg( 'stereo_params', default_value=['stereo_params']),
        LaunchArg( 'filter_params', default_value=['filter_params']),
        LaunchArg( 'namespace', default_value=['SM2'], description='Namespace of topics'),
        

        # Node definition
        Node(
            package='voris_log',
            executable='stereo_tools',
            name='stereo_tools',
            namespace=LaunchConfig('namespace'),
            remappings=[
                ('disparity_image',  LaunchConfig('disparity_image')),
                ('stereo_params', LaunchConfig('stereo_params')),
                ('filter_params', LaunchConfig('filter_params'))
            ]
        )
    ])
