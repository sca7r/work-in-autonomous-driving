import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point, TransformStamped
from std_msgs.msg import Bool
from adapt_envmod.env import EnvModel  
from adapt_msgs.msg import DetectedObjects, DetectedObject  
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class IntegrationHelperNode(Node):
    """Helper node for publishing test messages and broadcasting static transforms."""

    def __init__(self):
        super().__init__('test_helper_node')
        self.pose_publisher = self.create_publisher(PoseStamped, '/loc_pose', 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.stop_subscription = self.create_subscription(Bool, '/stop', self.stop_callback, 10)
        self.stop_received = False
        self.get_logger().info("Integration Helper Node initialized")

    def stop_callback(self, msg):
        """Callback to handle stop messages."""
        self.stop_received = msg.data

    def send_pose_message(self):
        """Send a pose message."""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = '9/base_link'
        pose_msg.pose = Pose(
            position=Point(x=1.0, y=2.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        self.pose_publisher.publish(pose_msg)

    def send_scan_message(self, ranges):
        """Send a laser scan message with the given ranges."""
        scan_msg = LaserScan()
        scan_msg.header.frame_id = '9/laser_frame'
        scan_msg.angle_min = -1.57
        scan_msg.angle_max = 1.57
        scan_msg.angle_increment = 0.1
        scan_msg.time_increment = 0.1
        scan_msg.range_min = 0.0
        scan_msg.range_max = 10.0
        scan_msg.ranges = ranges
        self.scan_publisher.publish(scan_msg)

@pytest.fixture(scope="module")
def env_model_node():
    """Fixture to create and return an instance of EnvModel node."""
    rclpy.init()
    node = EnvModel()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_env_model_integration(env_model_node):
    """Integration test for the EnvModel node."""
    helper_node = IntegrationHelperNode()
    executor = SingleThreadedExecutor()

    executor.add_node(env_model_node)
    executor.add_node(helper_node)

    try:
        # Send pose and scan messages
        helper_node.send_pose_message()
        helper_node.send_scan_message([1.0] * 32)  # No obstacles

        # Run the executor to process messages
        for _ in range(5):
            executor.spin_once(timeout_sec=1)

        assert not helper_node.stop_received, "Stop message was incorrectly received."

    finally:
        executor.shutdown()
        helper_node.destroy_node()

if __name__ == "__main__":
    pytest.main()

