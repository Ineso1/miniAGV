#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat
import math

import csv
import os
from datetime import datetime


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.subscription = self.create_subscription(Twist, 'vel_raw', self.vel_callback, 50)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Flag to enable or disable CSV logging
        self.enable_csv_logging = True

        if self.enable_csv_logging:
            log_dir = '/root/Documents/robot/miniAGV/agv_ws/data'
            os.makedirs(log_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_file_path = os.path.join(log_dir, f'velocity_log_{timestamp}.csv')

            self.csv_file = open(self.csv_file_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['vx', 'vy', 'angular', 'x', 'y', 'theta'])  # Include position in header

            self.get_logger().info(f'Logging to CSV file: {self.csv_file_path}')

    def vel_callback(self, msg: Twist):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        vx = msg.linear.x
        vy = msg.linear.y
        vth = msg.angular.z

        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta))
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta))
        
        delta_theta = vth

        self.x += delta_x * dt
        self.y += delta_y * dt
        self.theta += delta_theta * dt
        self.theta = self.normalize_angle(self.theta)

        q = euler2quat(0.0, 0.0, self.theta)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        odom.twist.twist.linear.x = delta_x
        odom.twist.twist.linear.y = delta_y
        odom.twist.twist.angular = msg.angular

        self.odom_pub.publish(odom)

        if self.enable_csv_logging:
            # Log velocities and positions (x, y, theta) to CSV
            self.csv_writer.writerow([vx, vy, vth, self.x, self.y, self.theta])
            self.csv_file.flush()

        self.get_logger().debug(f'Published and logged: vx={vx}, vy={vy}, angular={vth}, x={self.x}, y={self.y}, theta={self.theta}')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def destroy_node(self):
        if self.enable_csv_logging:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
