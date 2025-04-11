from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='controller',
            name='controller',
            output='screen'
        ),
        Node(
            package='control',
            executable='odometry',
            name='odometry',
            output='screen'
        ),
        Node(
            package='control',
            executable='trajectory',
            name='trajectory',
            output='screen',
        )
    ])