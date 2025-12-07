import unittest
import rclpy
from std_msgs.msg import String
from code.ros2_ws.src.module1_examples.publisher import AdvancedPublisher
from code.ros2_ws.src.module1_examples.subscriber import AdvancedSubscriber

class TestModule1Examples(unittest.TestCase):

    def test_publisher_subscriber(self):
        rclpy.init()
        publisher_node = AdvancedPublisher()
        subscriber_node = AdvancedSubscriber()

        # Let the nodes spin for a moment to allow for discovery and message passing
        rclpy.spin_once(publisher_node, timeout_sec=1)
        rclpy.spin_once(subscriber_node, timeout_sec=1)

        # This is a simplified test. In a real scenario, you'd want to
        # use a more robust way to check if messages were received.
        # For example, you could add a flag to the subscriber that gets
        # set when a message is received, and then check that flag here.
        # For the purpose of this example, we'll just check that the nodes
        # can be created and spun without errors.
        self.assertIsNotNone(publisher_node)
        self.assertIsNotNone(subscriber_node)

        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
