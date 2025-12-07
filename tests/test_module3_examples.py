import unittest
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from code.ros2_ws.src.module3_examples.vla_integration import VlaIntegrationNode

class TestModule3Examples(unittest.TestCase):

    def test_vla_integration_node(self):
        rclpy.init()
        vla_node = VlaIntegrationNode()

        # In a real test, you would publish mock image and text messages
        # and check if the action publisher publishes the expected action.
        # For this example, we'll just check that the node can be created
        # and spun without errors.
        self.assertIsNotNone(vla_node)
        rclpy.spin_once(vla_node, timeout_sec=1)

        vla_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
