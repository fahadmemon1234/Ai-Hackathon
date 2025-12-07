import unittest
import rclpy

from chapter2_examples.publisher import Publisher
from chapter2_examples.subscriber import Subscriber

class TestChapter2Examples(unittest.TestCase):

    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def test_publisher_creation(self):
        try:
            publisher_node = Publisher()
            publisher_node.destroy_node()
        except Exception as e:
            self.fail(f"Publisher node creation failed with {e}")

    def test_subscriber_creation(self):
        try:
            subscriber_node = Subscriber()
            subscriber_node.destroy_node()
        except Exception as e:
            self.fail(f"Subscriber node creation failed with {e}")

    def test_pub_sub_communication(self):
        # This is a placeholder test.
        # A more complete test would involve spinning the nodes and checking for message reception.
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
