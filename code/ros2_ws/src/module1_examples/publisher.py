import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AdvancedPublisher(Node):
    def __init__(self):
        super().__init__('advanced_publisher')
        self.publisher_ = self.create_publisher(String, 'advanced_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from advanced publisher: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    advanced_publisher = AdvancedPublisher()
    rclpy.spin(advanced_publisher)
    advanced_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
