import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        timer_period = 0.5  # seconds
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(timer_period, self.new_position_callback)
        self.i = 0

    def new_position_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main():
    print('Standing trajectory node')
    rclpy.init()
    trajectory_node = TrajectoryNode()
    rclpy.spin(trajectory_node)
    print('fuera')


if __name__ == '__main__':
    main()
