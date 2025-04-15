#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat
import math


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.subscription = self.create_subscription(Twist, 'vel_raw', self.vel_callback, 50)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def vel_callback(self, msg: Twist):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        vx = msg.linear.x
        vy = msg.linear.y
        vth = msg.angular.z

        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        q = euler2quat(0.0, 0.0, self.theta)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        odom.twist.twist = msg

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
