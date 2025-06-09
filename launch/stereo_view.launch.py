from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare the launch argument for 'SM'
        DeclareLaunchArgument(
            'SM',
            default_value='SM3',
            description='This is the sm argument'
        ),

        DeclareLaunchArgument('left_image', default_value=['left/image_raw'], description='stereo left image'),
        DeclareLaunchArgument('right_image', default_value=['right/image_raw'], description='stereo right image'),
        DeclareLaunchArgument('left_info', default_value=['left/camera_info'], description='left camera info'),
        DeclareLaunchArgument('right_info', default_value=['right/camera_info'], description='right camera info'),
        # Declare the launch argument for enabling the window
        DeclareLaunchArgument(
            'show_window',
            default_value= 'false',
            description='Enable or disable the image window'
        ),
        DeclareLaunchArgument(
            'rectificate_image',
            default_value= 'false',
            description='Enable or disable rectification of image'
        ),

        # Node definition
        Node(
            package='voris_log',
            executable='stereo_view',
            namespace=LaunchConfiguration('SM'),
            name='stereo_view',
            arguments=[
                PathJoinSubstitution(['/', LaunchConfiguration('SM'),LaunchConfiguration('left_info')]),
                PathJoinSubstitution(['/', LaunchConfiguration('SM'),LaunchConfiguration('right_info')])
                ],
            output='screen',
            remappings=[
                ('/camera_1/image_raw', LaunchConfiguration('left_image')),
                ('/camera_2/image_raw', LaunchConfiguration('right_image')),
                ('/camera_1/image_rect', [LaunchConfiguration('left_image'), TextSubstitution(text='_rect')]),
                ('/camera_2/image_rect', [LaunchConfiguration('right_image'), TextSubstitution(text='_rect')]),
            ],
            parameters=[
                {
                    'show_window': LaunchConfiguration('show_window'),  # Pass the argument value to the node
                    'rectificate_image': LaunchConfiguration('rectificate_image')
                }
            ]
        )
        ]
)