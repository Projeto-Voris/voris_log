import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments to specify YAML file paths for left and right camera
        DeclareLaunchArgument('yaml_left',
            default_value='/home/voris/ros2_ws/src/flir_camera_driver/spinnaker_camera_driver/config/16378750.yaml', 
            description='Path to the left camera YAML file'),
        DeclareLaunchArgument('yaml_right', 
            default_value='/home/voris/ros2_ws/src/flir_camera_driver/spinnaker_camera_driver/config/16378749.yaml',
            description='Path to the right camera YAML file'),

        DeclareLaunchArgument('left_frameID', default_value='/Passive/left_camera_link', description='Left frame ID'),
        DeclareLaunchArgument('right_frameID', default_value='/Passive/right_camera_link', description='right frame ID'),
        DeclareLaunchArgument('namespace', default_value='Passive', description='namespace'),

        # Node configuration
        Node(
            package='voris_log',  # replace with your actual package name
            executable='camerainfo_publisher.py',
            name='camera_info_publisher_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[LaunchConfiguration('yaml_left'), LaunchConfiguration('yaml_right')],
            parameters=[
                {'left_frameID': LaunchConfiguration('left_frameID')},
                {'right_frameID': LaunchConfiguration('right_frameID')}
            ],
            remappings=[
                ('left/camera_info', 'left/camera_info'),
                ('right/camera_info', 'right/camera_info')
            ]
        ),
    ])
