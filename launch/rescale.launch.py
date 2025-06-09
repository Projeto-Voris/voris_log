from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Define the ImageRescaler node
    return LaunchDescription([
    DeclareLaunchArgument('namespace', default_value='SM2', description='namespace of node'),
        Node(
            package='voris_log', # Replace with your package name
            executable='rescale',   # Name of the executable (from CMakeLists.txt)
            name='image_l',        # Node name
            namespace=LaunchConfiguration('namespace'),         # Namespace for the node
            output='screen',               # Print output to the screen
            remappings=[
                # Remap the subscriber topic
                ('image_in','/SM2/left/image_rect'),
                ('camera_info_in','/SM2/left/camera_info'),
                # Remap the publisher topic
                ('image_out','/teste/left/image_raw'),
                ('camera_info_out','/teste/left/camera_info')
            ]

        ),
        Node(
            package='voris_log', # Replace with your package name
            executable='rescale',   # Name of the executable (from CMakeLists.txt)
            name='image_r',         # Node name
            namespace=LaunchConfiguration('namespace'),         # Namespace for the node
            output='screen',               # Print output to the screen
            remappings=[
                # Remap the subscriber topic
                ('image_in','/SM2/right/image_rect'),
                ('camera_info_in','/SM2/right/camera_info'),
                # Remap the publisher topic
                ('image_out','/teste/right/image_raw'),
                ('camera_info_out','/teste/right/camera_info')
            ]
            )
    ])