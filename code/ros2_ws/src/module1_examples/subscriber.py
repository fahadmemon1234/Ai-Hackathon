import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AdvancedSubscriber(Node):
    def __init__(self):
        super().__init__('advanced_subscriber')
        self.subscription = self.create_subscription(
            String,
            'advanced_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    advanced_subscriber = AdvancedSubscriber()
    rclpy.spin(advanced_subscriber)
    advanced_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
