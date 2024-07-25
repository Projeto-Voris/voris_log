from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='voris_log',
            namespace='',
            executable='posearray_cb',
            name='posearray_sim_log'
        ),
        Node(
            package='voris_log',
            namespace='',
            executable='posestamped_cb',
            name='posestamped_sim_log'
        ),
    ])