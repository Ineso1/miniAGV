import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
from your_package_name.msg import RobotState
from geometry_msgs.msg import Vector3

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(RobotState, '/filtered_state', 10)

        self.dt = 0.1  # time step

        # State vector: [x, y, z, vx, vy, vz, ax, ay, az, wz, az_z]
        self.x = np.zeros((11, 1))

        # State covariance
        self.P = np.eye(11)

        # Define matrices
        self.A = np.eye(11)
        for i in range(3):  # Position-velocity
            self.A[i, i+3] = self.dt
            self.A[i, i+6] = 0.5 * self.dt ** 2
        for i in range(3, 6):  # Velocity-acceleration
            self.A[i, i+3] = self.dt
        self.A[9, 10] = self.dt  # wz - az_z

        self.B = np.zeros((11, 1))  # Not used here (no control input)

        self.Q = 0.01 * np.eye(11)  # Process noise
        self.R = 0.1 * np.eye(6)    # Measurement noise
        self.H = np.zeros((6, 11))
        self.H[0:3, 0:3] = np.eye(3)   # position
        self.H[3:6, 3:6] = np.eye(3)   # velocity

    def odom_callback(self, msg: Odometry):
        # Measurement vector [x, y, z, vx, vy, vz]
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]).reshape((6, 1))

        # Prediction
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

        # Update
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(11) - K @ self.H) @ self.P

        # Publish
        msg_out = RobotState()
        msg_out.position = Vector3(x=self.x[0,0], y=self.x[1,0], z=self.x[2,0])
        msg_out.velocity = Vector3(x=self.x[3,0], y=self.x[4,0], z=self.x[5,0])
        msg_out.acceleration = Vector3(x=self.x[6,0], y=self.x[7,0], z=self.x[8,0])
        msg_out.angular_velocity_z = self.x[9,0]
        msg_out.angular_acceleration_z = self.x[10,0]

        self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
