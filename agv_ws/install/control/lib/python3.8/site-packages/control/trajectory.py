#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from datetime import datetime
import csv
import os

from .soft_trajectory_generator import SoftTrajectoryGenerator


class TrajectoryNode(Node):
    """
    ROS 2 node that generates and publishes a smooth trajectory based on waypoints using the 
    SoftTrajectoryGenerator class. It publishes the desired position (x, y, theta) at a specified frequency 
    and logs the trajectory to a CSV file if enabled.

    Attributes:
        pose_pub (rclpy.publisher): Publisher for the desired robot pose (x, y, theta).
        timer (rclpy.timer): Timer that triggers publishing of desired pose at a set frequency.
        softTrajectory (SoftTrajectoryGenerator): Instance of the SoftTrajectoryGenerator to handle trajectory creation.
        enable_csv_logging (bool): Flag to enable or disable logging to CSV file.
        csv_file_path (str): Path to the CSV file for logging trajectory data.
        csv_file (file): Open file object to write trajectory data.
        csv_writer (csv.writer): CSV writer object to log trajectory data in the file.
    """

    def __init__(self):
        """
        Initializes the TrajectoryNode by setting up the publisher, timer, trajectory generator, and CSV logging.

        This function sets up the ROS 2 node with the following tasks:
        - Creating a publisher for the desired robot pose (`desired_pose` topic).
        - Setting up a timer to publish the desired pose at 10Hz (0.1 seconds).
        - Adding waypoints and generating the trajectory using the `SoftTrajectoryGenerator`.
        - Setting up CSV logging to track the robot's trajectory.
        """
        super().__init__('trajectory_node')

        # Create a publisher to send the desired pose (x, y, theta) to the 'desired_pose' topic
        self.pose_pub = self.create_publisher(Pose2D, 'desired_pose', 10)
        # Create a timer to call `publish_desired_pose` every 0.1 seconds (10Hz)
        self.timer = self.create_timer(0.1, self.publish_desired_pose)

        # Initialize trajectory generator
        self.softTrajectory = SoftTrajectoryGenerator()

        # Add waypoints for trajectory generation (position [x, y, theta], and time in seconds)
        self.softTrajectory.addWaypoint([0.0, 0.0, 0.0], 0)
        self.softTrajectory.addWaypoint([0.78, 0.0, 0.0], 5)
        self.softTrajectory.addWaypoint([0.78, 0.78, 0.0], 10)
        self.softTrajectory.addWaypoint([0.0, 0.78, 0.0], 15)
        self.softTrajectory.addWaypoint([0.0, 0.0, 0.0], 20)
        self.softTrajectory.addWaypoint([0.78, 0.0, 0.0], 28)

        # Generate the trajectory using the specified waypoints
        self.softTrajectory.generateTrajectories()

        # CSV logging setup
        self.enable_csv_logging = True
        if self.enable_csv_logging:
            # Specify the directory and file path for CSV logging
            log_dir = '/root/Documents/robot/miniAGV/agv_ws/data'  # Same as OdometryNode
            os.makedirs(log_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_file_path = os.path.join(log_dir, f'trajectory_log_{timestamp}.csv')
            self.csv_file = open(self.csv_file_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['x', 'y', 'theta'])  # Header for CSV file
            self.get_logger().info(f"Logging trajectory to: {self.csv_file_path}")

    def publish_desired_pose(self):
        """
        Publishes the current desired robot pose (x, y, theta) based on the trajectory and logs it to a CSV file.

        The desired pose is computed by calling `getNextState` from the `SoftTrajectoryGenerator` with 
        a fixed time step (0.1 seconds). The pose is then published to the 'desired_pose' topic, 
        and the data is logged to a CSV file if logging is enabled.
        """
        dt = 0.1  # Time step for trajectory update (10Hz)
        position, _ = self.softTrajectory.getNextState(dt)  # Get the next position and velocity

        # Create a Pose2D message to store the robot's desired pose
        pose_msg = Pose2D()
        pose_msg.x = float(position[0])
        pose_msg.y = float(position[1])
        pose_msg.theta = float(position[2])

        # Publish the pose message to the 'desired_pose' topic
        self.pose_pub.publish(pose_msg)

        # Log the pose to CSV if CSV logging is enabled
        if self.enable_csv_logging:
            self.csv_writer.writerow([pose_msg.x, pose_msg.y, pose_msg.theta])
            self.csv_file.flush()
            # Optional debug print
            print(f"Logged: x={pose_msg.x}, y={pose_msg.y}, theta={pose_msg.theta}")

    def destroy_node(self):
        """
        Cleans up the node by closing the CSV file before shutting down the node.
        """
        if self.enable_csv_logging:
            self.csv_file.close()  # Close the CSV file to ensure all data is written
        super().destroy_node()  # Call the base class's destroy_node to clean up the node

def main(args=None):
    """
    Initializes the ROS 2 client library and runs the TrajectoryNode.

    Args:
        args (list, optional): Arguments passed to the ROS 2 node (default is None).
    """
    rclpy.init(args=args)  # Initialize the ROS 2 client library
    node = TrajectoryNode()  # Create an instance of the TrajectoryNode
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()  # Shutdown the node
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()
