#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from math import pi
import smbus
from SunriseRobotLib import SunriseRobot

from std_msgs.msg import Int32, Bool, UInt8MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField


class MoveDrivers(Node):
    def __init__(self):
        super().__init__('move_drivers')

        self.RA2DE = 180 / pi
        self.car = SunriseRobot()
        self.car.set_car_type(6)
        self.car.create_receive_threading()

        # Parameters
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('Prefix', "")
        self.declare_parameter('xlinear_limit', 1.0)
        self.declare_parameter('ylinear_limit', 1.0)
        self.declare_parameter('angular_limit', 1.0)

        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
        self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
        self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
        self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value

        # I2C
        self.bus = smbus.SMBus(0)
        self.RGBLight_ctrl = True

        # Publishers
        self.imuPublisher = self.create_publisher(Imu, "imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "imu/mag", 100)
        self.odomPublisher = self.create_publisher(Twist, "vel_raw", 50)

        # Subscribers
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(UInt8MultiArray, "RGBLight", self.RGBLightcallback, 10)
        self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 10)

        # Timer
        self.create_timer(0.1, self.pub_data)

    def cmd_vel_callback(self, msg):
        self.car.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)

    def RGBLightcallback(self, msg):
        try:
            if len(msg.data) != 3:
                self.get_logger().warn("RGBLight expects 3 values: [R, G, B]")
                return

            r, g, b = msg.data

            self.bus.write_byte_data(0x0d, 0x03, r)  # Red
            self.bus.write_byte_data(0x0d, 0x04, g)  # Green
            self.bus.write_byte_data(0x0d, 0x05, b)  # Blue

            self.get_logger().info(f"Set RGB to: R={r}, G={g}, B={b}")
        except OSError as e:
            self.get_logger().error(f"I2C RGB write failed: {e}")

    def Buzzercallback(self, msg):
        state = 1 if msg.data else 0
        for _ in range(3):
            self.car.set_beep(state)

    def pub_data(self):
        now = Clock().now().to_msg()

        imu = Imu()
        mag = MagneticField()

        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()

        # IMU
        imu.header.stamp = now
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = -ax
        imu.linear_acceleration.y = az
        imu.linear_acceleration.z = ay
        imu.angular_velocity.x = -gx
        imu.angular_velocity.y = gz
        imu.angular_velocity.z = gy

        # Magnetometer
        mag.header.stamp = now
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = mx
        mag.magnetic_field.y = my
        mag.magnetic_field.z = mz

        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)

        # Velocity
        vx, vy, angular = self.car.get_motion_data()

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = angular

        self.odomPublisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MoveDrivers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
