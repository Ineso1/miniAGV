#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from geometry_msgs.msg import Twist
from SunriseRobotLib import SunriseRobot  # Hardware lib

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.car = SunriseRobot()
        self.car.set_car_type(6)
        self.car.create_receive_threading()

        self.vel_publisher = self.create_publisher(Twist, 'vel_raw', 50)
        self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        vx, vy, angular = self.car.get_motion_data()

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = angular

        self.vel_publisher.publish(twist)
        self.get_logger().debug(f'Published vel_raw: vx={vx}, vy={vy}, angular={angular}')


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
