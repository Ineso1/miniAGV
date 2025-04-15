#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math


class PIDController:
    def __init__(self, p, i, d, max_output, min_output):
        self.p = p
        self.i = i
        self.d = d
        self.max_output = max_output
        self.min_output = min_output
        self.integral_error = 0.0

    def compute(self, pos_error, vel_error, dt):
        if dt <= 0:
            return 0.0

        proportional = self.p * pos_error
        self.integral_error += pos_error * dt
        integral = self.i * self.integral_error
        derivative = self.d * vel_error

        output = proportional + integral + derivative
        return max(min(output, self.max_output), self.min_output)


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Subscribers
        self.create_subscription(Pose2D, 'desired_pose', self.desired_pose_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # PID controllers
        self.pid_x = PIDController(1.0, 0.0, 0.0, 1.0, -1.0)
        self.pid_y = PIDController(1.0, 0.0, 0.0, 1.0, -1.0)
        self.pid_theta = PIDController(2.0, 0.0, 0.1, 1.0, -1.0)

        # Internal state
        self.current_pose = Pose2D()
        self.current_twist = Twist()
        self.desired_pose = None
        self.desired_twist = Twist()
        self.last_time = self.get_clock().now()

    def odometry_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Use transforms3d to convert quaternion to yaw
        yaw, _, _ = quat2euler([ori.w, ori.x, ori.y, ori.z])

        self.current_pose.x = pos.x
        self.current_pose.y = pos.y
        self.current_pose.theta = yaw
        self.current_twist = msg.twist.twist  # Use `.twist`, not entire msg

    def desired_pose_callback(self, msg: Pose2D):
        self.desired_pose = msg

        # Zero desired velocities (pure position control)
        self.desired_twist = Twist()

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        # Calculate errors
        error_x = self.desired_pose.x - self.current_pose.x
        error_y = self.desired_pose.y - self.current_pose.y
        error_theta = self.normalize_angle(self.desired_pose.theta - self.current_pose.theta)

        error_vx = self.desired_twist.linear.x - self.current_twist.linear.x
        error_vy = self.desired_twist.linear.y - self.current_twist.linear.y
        error_omega = self.desired_twist.angular.z - self.current_twist.angular.z

        # PID output
        v_x = self.pid_x.compute(error_x, error_vx, dt)
        v_y = self.pid_y.compute(error_y, error_vy, dt)
        v_theta = self.pid_theta.compute(error_theta, error_omega, dt)

        # Publish Twist message
        twist = Twist()
        twist.linear.x = v_x
        twist.linear.y = v_y
        twist.angular.z = v_theta
        self.cmd_vel_pub.publish(twist)

        self.last_time = current_time

        self.get_logger().info(
            f"Pose error: x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}, theta={math.degrees(self.current_pose.theta):.2f}° "
        )

        # self.get_logger().info(
        #     f"Pose error: x={error_x:.2f}, y={error_y:.2f}, theta={math.degrees(error_theta):.2f}° | "
        #     f"cmd_vel: x={v_x:.2f}, y={v_y:.2f}, z={v_theta:.2f}"
        # )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
