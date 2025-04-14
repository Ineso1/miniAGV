
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # model_params = os.path.join(
    #     get_package_share_directory('puzzle_pkg'),
    #     'config',
    #     'controller_params.yaml'
    # )
    # model_params = os.path.join(
    #     get_package_share_directory('puzzle_pkg'),
    #     'config',
    #     'odometry_params.yaml'
    # )

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
            # output='screen'
        ),
        Node(
            package='control',
            executable='trajectory',
            name='trajectory',
            # output='screen',
        )
    ])