# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile

# class ControlNode(Node):
#     def __init__(self):
#         super().__init__('control_node')
        
#         qos_profile = QoSProfile(depth=10)
#         self.subscription = self.create_subscription(
#             Twist,
#             '/vel_raw',
#             self.vel_raw_callback,
#             qos_profile
#         )
#         self.subscription  # prevent unused variable warning
        
#         self.odometry_publisher = self.create_publisher(
#             Odometry,
#             '/odometry',
#             10  # alternatively, you can reuse a QoSProfile if desired
#         )
        
#         timer_period = 1.0  # seconds
#         self.timer = self.create_timer(timer_period, self.publish_odometry_callback)        
#         self.current_position = [0.0, 0.0, 0.0]  # x, y, z

#     def vel_raw_callback(self, msg: Twist):
#         self.get_logger().info(
#             f"Received Twist: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), "
#             f"angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})"
#         )

#     def publish_odometry_callback(self):
#         odom_msg = Odometry()
        
#         odom_msg.pose.pose.position.x = self.current_position[0]
#         odom_msg.pose.pose.position.y = self.current_position[1]
#         odom_msg.pose.pose.position.z = self.current_position[2]
        
#         self.odometry_publisher.publish(odom_msg)
#         self.get_logger().info("Published odometry message.")

# def main():
#     print('Hi from control.')
#     rclpy.init()
#     control_node = ControlNode()
#     rclpy.spin(control_node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()








import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Twist

class RoverController(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0

        self.rot_x      = 0.0
        self.rot_y      = 0.0
        self.rot_z      = 0.0

        self.thetaeint = 0.0

        self.dt  = 0.01

        self.kpr = 3
        self.kpt = 1
        self.kir = 0.03

        self.pose_sub = self.create_subscription(Twist, "vel_raw", self.pose_callback, 10)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.posx_pub = self.create_publisher(Float64, "/positionx", 10)
        self.posy_pub = self.create_publisher(Float64, "/positiony", 10)
        self.angle_pub = self.create_publisher(Float32, "/anglecuh", 10)

    def response_calc(self, pos_x, pos_y, curr_theta, xd, yd):
        # Position control
        x = pos_x
        y = pos_y
        theta = curr_theta
        thetad = np.arctan2((yd-y), (xd-x))

        thetae = theta - thetad
        if thetae > np.pi:
            thetae = thetae - 2*np.pi
        elif thetae < -np.pi:
            thetae = thetae + 2*np.pi
        #Thetae.append(thetae)
        #Time.append(t)

        # PI controller for orientation
        self.thetaeint += thetae*self.dt
        w = -self.kpr*thetae - self.kir*self.thetaeint

        d = np.sqrt((xd-x)**2 + (yd-y)**2)
        V = self.kpt *d

        return V, w

    def pose_callback(self, msg):
        #get current linear speed and angular speed
        velocidad_x = msg.linear.x
        velocidad_y = msg.linear.y
        omega = msg.angular.z

        self.position_x += velocidad_x*self.dt
        self.position_y += velocidad_y*self.dt
        self.position_z = msg.linear.z
        self.rot_x      = msg.angular.x
        self.rot_y      = msg.angular.y
        self.rot_z      += omega*self.dt
        print(self.position_x)
        vel, turn = self.response_calc(self.position_x, self.position_y, self.rot_z, 0.7,0.5)

        twist_msg = Twist()
        #twist_msg.linear.x = vel
        twist_msg.linear.x = np.clip(vel, -0.05, 0.05)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        #twist_msg.angular.z = turn
        twist_msg.angular.z = np.clip(turn, -0.05, 0.05)

        self.twist_pub.publish(twist_msg)
        #self.posx_pub.publish(self.position_x)
        #self.posy_pub.publish(self.position_y)
        #self.angle_pub.publish(self.rot_z)


def main(args=None):
    print("Standing controller node")
    rclpy.init(args=args)
    roverController = RoverController()
    rclpy.spin(roverController)

if __name__=='__main__':
    main()
