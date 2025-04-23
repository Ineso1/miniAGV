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
    """
    ROS 2 node for controlling the motion and sensors of a robot. This node interacts with the SunriseRobot 
    hardware to handle motor movement, IMU sensor data, and RGB light control. It subscribes to commands 
    for movement, light settings, and buzzer control while publishing IMU and motion data.

    Attributes:
        RA2DE (float): Constant to convert between radians and degrees.
        car (SunriseRobot): Instance of the SunriseRobot class used to control robot motion and sensors.
        imu_link (str): The frame id to associate with IMU data.
        Prefix (str): Prefix string used in messages (currently unused).
        xlinear_limit (float): Linear velocity limit for the robot's x-axis.
        ylinear_limit (float): Linear velocity limit for the robot's y-axis.
        angular_limit (float): Angular velocity limit for the robot's rotation.
        bus (smbus.SMBus): I2C bus object to communicate with hardware devices like RGB lights.
        RGBLight_ctrl (bool): Flag to control RGB light.
        imuPublisher (rclpy.publisher): Publisher to send IMU data on "imu/data_raw" topic.
        magPublisher (rclpy.publisher): Publisher to send magnetic field data on "imu/mag" topic.
        odomPublisher (rclpy.publisher): Publisher to send velocity data on "vel_raw" topic.
    """

    def __init__(self):
        """
        Initializes the MoveDrivers node by setting up publishers, subscribers, and parameters for robot control.
        
        The constructor performs the following:
        - Initializes the SunriseRobot instance to control the car's motion.
        - Declares parameters for IMU link, prefix, linear velocity limits, and angular velocity limit.
        - Sets up I2C communication for controlling the RGB light.
        - Creates publishers to publish IMU, magnetic field, and velocity data.
        - Subscribes to topics for controlling velocity, RGB light, and buzzer.
        - Starts a timer to periodically publish sensor data.
        """
        super().__init__('move_drivers')

        self.RA2DE = 180 / pi  # Conversion factor from radians to degrees
        self.car = SunriseRobot()
        self.car.set_car_type(6)  # Set the car type (assuming '6' is a specific type)
        self.car.create_receive_threading()  # Start receiving data in a separate thread

        # Declare parameters for configuration
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('Prefix', "")
        self.declare_parameter('xlinear_limit', 1.0)
        self.declare_parameter('ylinear_limit', 1.0)
        self.declare_parameter('angular_limit', 1.0)

        # Get parameter values
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
        self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
        self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
        self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value

        # Set up I2C bus communication for RGB light control
        self.bus = smbus.SMBus(0)
        self.RGBLight_ctrl = True  # Flag to enable RGB light control

        # Publishers for IMU, magnetic field, and velocity data
        self.imuPublisher = self.create_publisher(Imu, "imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "imu/mag", 100)
        self.odomPublisher = self.create_publisher(Twist, "vel_raw", 50)

        # Subscribers for velocity commands, RGB light control, and buzzer control
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(UInt8MultiArray, "RGBLight", self.RGBLightcallback, 10)
        self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 10)

        # Timer to periodically publish sensor data
        self.create_timer(0.1, self.pub_data)

    def cmd_vel_callback(self, msg):
        """
        Callback function for receiving velocity commands.

        This function is called when a message is received on the 'cmd_vel' topic. It updates the robot's motion 
        using the linear and angular velocities provided in the message.

        Args:
            msg (Twist): Velocity command message containing linear and angular velocities.
        """
        self.car.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)

    def RGBLightcallback(self, msg):
        """
        Callback function for controlling the RGB light.

        This function is called when a message is received on the 'RGBLight' topic. The message should contain 
        three values representing the red, green, and blue color intensities (0-255) for the RGB light.

        Args:
            msg (UInt8MultiArray): RGB color data containing 3 values [R, G, B].
        """
        try:
            if len(msg.data) != 3:
                self.get_logger().warn("RGBLight expects 3 values: [R, G, B]")
                return

            r, g, b = msg.data

            # Write the RGB values to the I2C bus (assuming the address and registers are correct)
            self.bus.write_byte_data(0x0d, 0x03, r)  # Red
            self.bus.write_byte_data(0x0d, 0x04, g)  # Green
            self.bus.write_byte_data(0x0d, 0x05, b)  # Blue

            self.get_logger().info(f"Set RGB to: R={r}, G={g}, B={b}")
        except OSError as e:
            self.get_logger().error(f"I2C RGB write failed: {e}")

    def Buzzercallback(self, msg):
        """
        Callback function for controlling the buzzer.

        This function is called when a message is received on the 'Buzzer' topic. If the message contains a 
        'True' value, the buzzer will be turned on. If 'False', the buzzer will be turned off. The buzzer will
        beep 3 times to indicate the state change.

        Args:
            msg (Bool): A boolean indicating whether to activate or deactivate the buzzer.
        """
        state = 1 if msg.data else 0
        for _ in range(3):  # Beep 3 times
            self.car.set_beep(state)

    def pub_data(self):
        """
        Publishes IMU data, magnetic field data, and velocity data.

        This function collects data from the robot's sensors (accelerometer, gyroscope, magnetometer, and motion 
        data) and publishes them to the corresponding ROS topics: "imu/data_raw", "imu/mag", and "vel_raw".

        The function also uses the system clock to timestamp the sensor data.
        """
        now = Clock().now().to_msg()  # Get the current timestamp

        imu = Imu()
        mag = MagneticField()

        # Get accelerometer, gyroscope, and magnetometer data from the robot
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()

        # IMU Data
        imu.header.stamp = now
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = -ax  # Negate to match frame of reference
        imu.linear_acceleration.y = az
        imu.linear_acceleration.z = ay
        imu.angular_velocity.x = -gx  # Negate to match frame of reference
        imu.angular_velocity.y = gz
        imu.angular_velocity.z = gy

        # Magnetic Field Data
        mag.header.stamp = now
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = mx
        mag.magnetic_field.y = my
        mag.magnetic_field.z = mz

        # Publish the IMU and magnetic field data
        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)

        # Velocity Data
        vx, vy, angular = self.car.get_motion_data()

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = angular

        # Publish the velocity data
        self.odomPublisher.publish(twist)


def main(args=None):
    """
    Initializes the ROS 2 client library, creates the MoveDrivers node, and spins it.

    This function initializes the ROS 2 client library, creates an instance of the MoveDrivers node, 
    and runs the ROS 2 event loop to keep the node active. Upon shutdown, it ensures the robot stops moving.

    Args:
        args (list, optional): Arguments passed to the ROS 2 node (default is None).
    """
    rclpy.init(args=args)  # Initialize the ROS 2 client library
    node = MoveDrivers()  # Create an instance of the MoveDrivers node

    try:
        rclpy.spin(node)  # Keep the node running
    finally:
        # Stop the robot before shutting down
        node.get_logger().info("Shutting down, stopping robot...")
        node.car.set_car_motion(0.0, 0.0, 0.0)  # Stop the robot
        node.destroy_node()  # Cleanup the node
        rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()
