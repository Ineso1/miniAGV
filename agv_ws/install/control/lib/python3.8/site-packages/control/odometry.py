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
    """
    This node calculates and publishes the robot's odometry based on incoming velocity commands (Twist messages).

    Attributes:
        x (float): The current x-coordinate of the robot.
        y (float): The current y-coordinate of the robot.
        theta (float): The current orientation (angle) of the robot.
        last_time (rclpy.time.Time): The time of the last odometry calculation.
        enable_csv_logging (bool): Flag to enable or disable CSV logging of velocity and odometry data.
        csv_file_path (str): The file path for the CSV log.
        csv_file (file object): The CSV file object to write data to.
        csv_writer (csv.writer): CSV writer object to log data in CSV format.
    """

    def __init__(self):
        """
        Initializes the OdometryNode. Subscribes to the 'vel_raw' topic for Twist messages 
        and sets up the 'odom' topic for publishing Odometry messages.

        Additionally, initializes logging to a CSV file with velocity and position data.
        """
        super().__init__('odometry_node')

        # Create subscription to raw velocity commands (Twist messages)
        self.subscription = self.create_subscription(Twist, 'vel_raw', self.vel_callback, 50)
        
        # Create publisher to publish the odometry data (Odometry messages)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)

        # Initialize position and orientation (x, y, theta) and last timestamp
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Flag to enable or disable CSV logging
        self.enable_csv_logging = True

        if self.enable_csv_logging:
            # Directory for logging the data
            log_dir = '/root/Documents/robot/miniAGV/agv_ws/data'
            os.makedirs(log_dir, exist_ok=True)

            # Generate the timestamped filename for the CSV log
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_file_path = os.path.join(log_dir, f'velocity_log_{timestamp}.csv')

            # Open CSV file and prepare the writer
            self.csv_file = open(self.csv_file_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['vx', 'vy', 'angular', 'x', 'y', 'theta'])  # Include position in header

            self.get_logger().info(f'Logging to CSV file: {self.csv_file_path}')

    def vel_callback(self, msg: Twist):
        """
        Callback function to handle incoming Twist messages, calculate the new odometry 
        based on the velocity commands, and publish the new Odometry message. If CSV logging 
        is enabled, it also logs the velocity and position data to a CSV file.

        Args:
            msg (Twist): The velocity command message with linear and angular velocities.
        """
        # Get the current time and calculate the time difference since the last update
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert time to seconds
        self.last_time = current_time

        # Extract linear and angular velocities from the incoming message
        vx = msg.linear.x
        vy = msg.linear.y
        vth = msg.angular.z

        # Compute the change in position and orientation
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta))
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta))
        
        # The change in orientation is simply the angular velocity multiplied by the time delta
        delta_theta = vth

        # Update the robot's position and orientation
        self.x += delta_x * dt
        self.y += delta_y * dt
        self.theta += delta_theta * dt
        self.theta = self.normalize_angle(self.theta)  # Normalize theta to be within -pi to pi

        # Convert the orientation to a quaternion
        q = euler2quat(0.0, 0.0, self.theta)

        # Create and populate the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position (x, y) and orientation (quaternion)
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        
        # Linear and angular velocity
        odom.twist.twist.linear.x = delta_x
        odom.twist.twist.linear.y = delta_y
        odom.twist.twist.angular = msg.angular

        # Publish the Odometry message
        self.odom_pub.publish(odom)

        # Log the velocity and position data to CSV if logging is enabled
        if self.enable_csv_logging:
            self.csv_writer.writerow([vx, vy, vth, self.x, self.y, self.theta])
            self.csv_file.flush()

        # Log for debugging purposes
        self.get_logger().debug(f'Published and logged: vx={vx}, vy={vy}, angular={vth}, x={self.x}, y={self.y}, theta={self.theta}')

    def normalize_angle(self, angle):
        """
        Normalizes an angle to be within the range -pi to pi.

        Args:
            angle (float): The angle to normalize (in radians).

        Returns:
            float: The normalized angle within the range -pi to pi.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def destroy_node(self):
        """
        Cleans up the node before shutdown, closing the CSV file if logging is enabled.
        """
        if self.enable_csv_logging:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    """
    Initializes the ROS 2 Python client library, creates an instance of the OdometryNode, 
    and starts spinning to process incoming messages.

    Args:
        args (list, optional): Command line arguments (default is None).
    """
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
