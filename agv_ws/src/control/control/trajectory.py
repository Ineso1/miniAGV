#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from datetime import datetime
import csv
import os

from .soft_trajectory_generator import SoftTrajectoryGenerator


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.pose_pub = self.create_publisher(Pose2D, 'desired_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_desired_pose)  # 10Hz

        # Initialize trajectory generator
        self.softTrajectory = SoftTrajectoryGenerator()

        # Add waypoints: [x, y, theta]
        self.softTrajectory.addWaypoint([0.0, 0.0, 0.0], 0)
        self.softTrajectory.addWaypoint([0.0, 0.5, 0.0], 5)
        self.softTrajectory.addWaypoint([0.5, 0.5, 1.57], 10)

        self.softTrajectory.generateTrajectories()

        # CSV logging setup
        self.enable_csv_logging = True
        if self.enable_csv_logging:
            log_dir = '/root/Documents/robot/miniAGV/agv_ws/data'  # Same as OdometryNode
            os.makedirs(log_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_file_path = os.path.join(log_dir, f'trajectory_log_{timestamp}.csv')
            self.csv_file = open(self.csv_file_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['x', 'y', 'theta'])
            self.get_logger().info(f"Logging trajectory to: {self.csv_file_path}")

    def publish_desired_pose(self):
        dt = 0.1
        position, _ = self.softTrajectory.getNextState(dt)

        pose_msg = Pose2D()
        pose_msg.x = float(position[0])
        pose_msg.y = float(position[1])
        pose_msg.theta = float(position[2])
        self.pose_pub.publish(pose_msg)

        # Log to CSV if enabled
        if self.enable_csv_logging:
            self.csv_writer.writerow([pose_msg.x, pose_msg.y, pose_msg.theta])
            self.csv_file.flush()
            print(f"Logged: x={pose_msg.x}, y={pose_msg.y}, theta={pose_msg.theta}")  # Debug print

    def destroy_node(self):
        if self.enable_csv_logging:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
