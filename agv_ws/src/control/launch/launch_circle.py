import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory  # Import this

def generate_launch_description():
    # Get the absolute path of the parameter file
    pid_yaml_file = os.path.join(
        get_package_share_directory('control'), 'params', 'pid.yaml')

    # Log the path for debugging purposes
    print(f"Attempting to load PID YAML file from: {pid_yaml_file}")
    
    # Ensure the file path is correct and exists
    if not os.path.isfile(pid_yaml_file):
        raise FileNotFoundError(f"PID config file not found at {pid_yaml_file}")
    
    # Return launch description with the parameters
    return LaunchDescription([
        Node(
            package='car_drivers',
            executable='move_drivers',
            name='move_drivers',
            output='screen'
        ),
        Node(
            package='control',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{'pid_config': pid_yaml_file}]  # <-- this is the fix
        ),
        Node(
            package='control',
            executable='odometry',
            name='odometry',
            output='screen'
        ),
        Node(
            package='control',
            executable='circle_trajectory',
            name='circle_trajectory',
            output='screen',
        )
    ])
