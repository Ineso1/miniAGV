#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from math import pi
import smbus
from SunriseRobotLib import SunriseRobot

from std_msgs.msg import Int32, Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField

class MoveDrivers(Node):
    def __init__(self):
        super().__init__('move_drivers')

        self.RA2DE = 180 / pi
        self.car = SunriseRobot()
        self.car.set_car_type(6)
        self.car.create_receive_threading()

        # Declare parameters
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

        # Setup I2C
        self.bus = smbus.SMBus(0)
        self.RGBLight_ctrl = True

        # Publishers
        self.EdiPublisher = self.create_publisher(Float32, "edition", 100)
        self.volPublisher = self.create_publisher(Float32, "voltage", 100)
        self.velPublisher = self.create_publisher(Twist, "vel_raw", 50)
        self.imuPublisher = self.create_publisher(Imu, "imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "imu/mag", 100)

        # Subscribers
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Int32, "RGBLight", self.RGBLightcallback, 10)
        self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 10)

        # Timer
        self.create_timer(0.1, self.pub_data)

    def cmd_vel_callback(self, msg):
        self.car.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)

    def RGBLightcallback(self, msg):
        if msg.data == 5:
            self.bus.write_byte_data(0x0d, 0x07, 0 if self.RGBLight_ctrl else 1)
            self.RGBLight_ctrl = not self.RGBLight_ctrl
        else:
            self.bus.write_byte_data(0x0d, 0x04, msg.data)

    def Buzzercallback(self, msg):
        state = 1 if msg.data else 0
        for _ in range(3):
            self.car.set_beep(state)

    def pub_data(self):
        now = Clock().now().to_msg()

        imu = Imu()
        twist = Twist()
        battery = Float32()
        edition = Float32()
        mag = MagneticField()

        edition.data = float(self.car.get_version())
        battery.data = float(self.car.get_battery_voltage())

        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()
        vx, vy, angular = self.car.get_motion_data()

        # Accelerometer
        imu.header.stamp = now
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = -ax
        imu.linear_acceleration.y = az
        imu.linear_acceleration.z = ay

        # Gyroscope
        imu.angular_velocity.x = -gx
        imu.angular_velocity.y = gz
        imu.angular_velocity.z = gy

        # Magnetometer
        mag.header.stamp = now
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = mx
        mag.magnetic_field.y = my
        mag.magnetic_field.z = mz

        # Velocity
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = angular

        self.EdiPublisher.publish(edition)
        self.volPublisher.publish(battery)
        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.velPublisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MoveDrivers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
