#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math
import time

class CircleTrajectoryNode(Node):

    def __init__(self):
        super().__init__('circle_trajectory_node')

        self.pose_pub = self.create_publisher(Pose2D, 'desired_pose', 10)
        self.timer = self.create_timer(0.05, self.publish_desired_pose)

        # Parámetros del círculo
        self.radius = 1.0  # radio del círculo
        self.angular_speed = 0.1  # radianes por segundo
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def publish_desired_pose(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        t = current_time - self.start_time
        theta = self.angular_speed * t

        pose_msg = Pose2D()
        pose_msg.x = self.radius * math.cos(theta)
        pose_msg.y = self.radius * math.sin(theta)
        pose_msg.theta = theta + math.pi / 2  # para que apunte tangencialmente
        # pose_msg.theta = 0.0  # para que apunte tangencialmente


        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f"Publicando: x={pose_msg.x:.2f}, y={pose_msg.y:.2f}, theta={pose_msg.theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = CircleTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
