from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the ImageRescaler node
    return LaunchDescription([
    Node(
        package='voris_log', # Replace with your package name
        executable='rescale',   # Name of the executable (from CMakeLists.txt)
        name='image_l',         # Node name
        output='screen',               # Print output to the screen
        remappings=[
            # Remap the subscriber topic
            ('image_in','/SM2/left/image_raw'),
            # Remap the publisher topic
            ('image_out','/SM2/left/image_rescaled')
        ]

    ),
    Node(
        package='voris_log', # Replace with your package name
        executable='rescale',   # Name of the executable (from CMakeLists.txt)
        name='image_r',         # Node name
        output='screen',               # Print output to the screen
        remappings=[
            # Remap the subscriber topic
            ('image_in','/SM2/right/image_raw'),
            # Remap the publisher topic
            ('image_out','/SM2/right/image_rescaled')
        ]
        )
])