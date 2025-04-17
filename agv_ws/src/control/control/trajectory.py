#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import csv
import os
from datetime import datetime


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.pose_pub = self.create_publisher(Pose2D, 'desired_pose', 10)
        self.timer = self.create_timer(0.5, self.publish_desired_pose)

        # Desired pose
        self.x = 0.0
        self.y = 0.5
        self.theta = 1.57  # ~90 degrees

        # CSV logging flag
        self.enable_csv_logging = True

        if self.enable_csv_logging:
            log_dir = '/root/Documents/robot/miniAGV/agv_ws/data'
            os.makedirs(log_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_file_path = os.path.join(log_dir, f'trajectory_log_{timestamp}.csv')
            self.csv_file = open(self.csv_file_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['x', 'y', 'theta'])
            self.get_logger().info(f"Logging trajectory to: {self.csv_file_path}")

    def publish_desired_pose(self):
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta

        self.pose_pub.publish(pose_msg)

        if self.enable_csv_logging:
            self.csv_writer.writerow([self.x, self.y, self.theta])
            self.csv_file.flush()

        # Optional debug output
        # self.get_logger().info(f"Publishing desired_pose: x={self.x}, y={self.y}, theta={self.theta}")

    def destroy_node(self):
        if self.enable_csv_logging:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    print('Starting trajectory node...')
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print('Shutting down trajectory node.')


if __name__ == '__main__':
    main()
