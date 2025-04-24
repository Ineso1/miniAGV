#!/usr/bin/env python3

##
# @file controller.py
# @brief ROS2 Control Node using PID controllers for miniAGV
# @author eq1
# @date 2025-04-22

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math
import csv
import os
from datetime import datetime

from paselbat_interfaces.msg import CustomPoseMsg


##
# @class PIDController
# @brief Implements a basic PID controller
class PIDController:
    ##
    # @brief Constructor for PIDController
    # @param p Proportional gain
    # @param i Integral gain
    # @param d Derivative gain
    # @param max_output Maximum control output
    # @param min_output Minimum control output
    def __init__(self, p, i, d, max_output, min_output):
        self.p = p
        self.i = i
        self.d = d
        self.max_output = max_output
        self.min_output = min_output
        self.integral_error = 0.0

    ##
    # @brief Compute the control output using PID formula
    # @param pos_error Position error
    # @param vel_error Velocity error
    # @param dt Time step
    # @return Clamped control output
    def compute(self, pos_error, vel_error, dt):
        if dt <= 0:
            return 0.0

        proportional = self.p * pos_error
        self.integral_error += pos_error * dt
        integral = self.i * self.integral_error
        derivative = self.d * vel_error

        output = proportional + integral + derivative
        return max(min(output, self.max_output), self.min_output)


##
# @class ControlNode
# @brief ROS2 Node implementing a PID-based motion controller
class ControlNode(Node):
    ##
    # @brief Constructor for ControlNode
    def __init__(self):
        super().__init__('control_node')

        # Enable or disable CSV logging
        self.enable_csv_logging = True

        if self.enable_csv_logging:
            log_dir = '/root/Documents/robot/miniAGV/agv_ws/data'
            os.makedirs(log_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_file_path = os.path.join(log_dir, f'control_log_{timestamp}.csv')
            self.csv_file = open(self.csv_file_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['error_x', 'error_y', 'error_theta', 'v_x', 'v_y', 'v_theta'])
            self.get_logger().info(f"Logging control data to: {self.csv_file_path}")

        # Subscribers
        self.create_subscription(Pose2D, 'desired_pose', self.desired_pose_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.custom_pos_pub = self.create_publisher(CustomPoseMsg, 'posecustom', 10)

        # PID controllers
        self.pid_x = PIDController(1.5, 0.4, 0.2, 1.0, -1.0)
        self.pid_y = PIDController(1.5, 0.4, 0.2, 1.0, -1.0)
        self.pid_theta = PIDController(3.5, 0.3, 0.2, 1.0, -1.0)

        # Internal state
        self.current_pose = Pose2D()
        self.current_twist = Twist()
        self.desired_pose = None
        self.desired_twist = Twist()
        self.last_time = self.get_clock().now()

        # Thresholds
        self.xy_threshold = 0.01
        self.theta_threshold = 0.01

        # Timer for control loop (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)

    ##
    # @brief Callback for odometry updates
    # @param msg Odometry message
    def odometry_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        _, _, yaw = quat2euler([ori.w, ori.x, ori.y, ori.z])
        self.current_pose.x = pos.x
        self.current_pose.y = pos.y
        self.current_pose.theta = yaw
        self.current_twist.linear = msg.twist.twist.linear
        self.current_twist.angular = msg.twist.twist.angular

    ##
    # @brief Callback for desired pose updates
    # @param msg Desired pose message
    def desired_pose_callback(self, msg: Pose2D):
        self.desired_pose = msg
        self.desired_twist = Twist()

    ##
    # @brief Main control loop callback
    def control_loop(self):
        if self.desired_pose is None:
            return  # Don't control until we have a desired pose

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        # Compute errors
        error_x = self.desired_pose.x - self.current_pose.x
        error_y = self.desired_pose.y - self.current_pose.y
        error_theta = self.normalize_angle(self.desired_pose.theta) - self.normalize_angle(self.current_pose.theta)

        error_vx = self.desired_twist.linear.x - self.current_twist.linear.x
        error_vy = self.desired_twist.linear.y - self.current_twist.linear.y
        error_omega = self.desired_twist.angular.z - self.current_twist.angular.z

        # Compute control outputs
        v_x = self.pid_x.compute(error_x, error_vx, dt)
        v_y = self.pid_y.compute(error_y, error_vy, dt)
        v_theta = self.pid_theta.compute(error_theta, error_omega, dt)

        # Apply thresholds
        if abs(error_x) < self.xy_threshold:
            v_x = 0.0
        if abs(error_y) < self.xy_threshold:
            v_y = 0.0
        if abs(error_theta) < self.theta_threshold:
            v_theta = 0.0

        vx_body = v_x * math.cos(self.current_pose.theta) + v_y * math.sin(self.current_pose.theta)
        vy_body = -v_x * math.sin(self.current_pose.theta) + v_y * math.cos(self.current_pose.theta)

        # Publish command
        twist = Twist()
        twist.linear.x = vx_body
        twist.linear.y = vy_body
        twist.angular.z = v_theta
        self.cmd_vel_pub.publish(twist)

        custompose = CustomPoseMsg()
        custompose.position.x = vx_body
        self.custom_pos_pub.publish(custompose)

        # Log to CSV if enabled
        if self.enable_csv_logging:
            self.csv_writer.writerow([error_x, error_y, error_theta, v_x, v_y, v_theta])
            self.csv_file.flush()

        self.get_logger().info(
            f"Pose: x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}, θ={math.degrees(self.current_pose.theta):.2f}°"
        )

        self.last_time = current_time

    ##
    # @brief Normalize an angle to [-π, π]
    # @param angle Angle in radians
    # @return Normalized angle
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    ##
    # @brief Custom cleanup on node destruction
    def destroy_node(self):
        if self.enable_csv_logging:
            self.csv_file.close()
        super().destroy_node()


##
# @brief Main entry point
# @param args Command line arguments
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
