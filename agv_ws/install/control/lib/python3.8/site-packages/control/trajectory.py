#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.pose_pub = self.create_publisher(Pose2D, 'desired_pose', 10)
        self.timer = self.create_timer(0.5, self.publish_desired_pose)

        # Define your desired point and angle
        self.x = 0.0   # Desired X position
        self.y = 0.0   # Desired Y position
        self.theta = 0.0  # Desired orientation (yaw) in radians (about 90 degrees)

    def publish_desired_pose(self):
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta

        self.pose_pub.publish(pose_msg)
        # self.get_logger().info(f"Publishing desired_pose: x={self.x}, y={self.y}, theta={self.theta}")


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
