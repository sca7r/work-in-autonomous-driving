import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from adapt_loc.localization import SimpleLocalization
from mocap_msgs.msg import RigidBodies, RigidBody
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Vector3

class IntegrationHelperNode(Node):
    """Helper node for verifying messages received from SimpleLocalization."""
    def __init__(self):
        super().__init__('test_integration')
        self.pose_subscription = self.create_subscription(
            PoseStamped, '/loc_pose', self.pose_callback, 10)
        self.euler_subscription = self.create_subscription(
            Vector3, '/euler_angles', self.euler_callback, 10)
        self.received_pose = False
        self.received_euler = False

    def pose_callback(self, msg):
        """Callback function to handle received PoseStamped messages."""
        self.get_logger().info(f"Pose received: {msg}")
        self.received_pose = True

    def euler_callback(self, msg):
        """Callback function to handle received Vector3 messages."""
        self.get_logger().info(f"Euler angles received: {msg}")
        self.received_euler = True

@pytest.fixture(scope="module", autouse=True)
def rclpy_setup_teardown():
    """Initialize and shutdown rclpy for the duration of the tests."""
    rclpy.init()
    yield
    rclpy.shutdown()

def test_localization_integration(rclpy_setup_teardown):
    """Integration test for SimpleLocalization node."""
    localization_node = SimpleLocalization()
    helper_node = IntegrationHelperNode()
    executor = SingleThreadedExecutor()
    executor.add_node(localization_node)
    executor.add_node(helper_node)

    # Simulate receiving a RigidBodies message
    pose = Pose()
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    pose.position.x = 1.12345
    pose.position.y = 2.12345
    pose.position.z = 3.12345

    body = RigidBody()
    body.pose = pose
    body.rigid_body_name = "9"

    rigid_bodies_msg = RigidBodies()
    rigid_bodies_msg.rigidbodies.append(body)

    localization_node.position_callback(rigid_bodies_msg)

    # Run the executor for a longer time to process the message
    for _ in range(5):
        executor.spin_once(timeout_sec=1)

    assert helper_node.received_pose, "PoseStamped message was not received"
    assert helper_node.received_euler, "Euler angles message was not received"

    # Test with a different rigid_body_name to ensure no messages are published
    body.rigid_body_name = "10"
    rigid_bodies_msg.rigidbodies[0] = body
    helper_node.received_pose = False
    helper_node.received_euler = False
    localization_node.position_callback(rigid_bodies_msg)
    for _ in range(5):
        executor.spin_once(timeout_sec=1)

    assert not helper_node.received_pose, "PoseStamped message should not have been received"
    assert not helper_node.received_euler, "Euler angles message should not have been received"
    
    # Clean up nodes
    executor.shutdown()
    localization_node.destroy_node()
    helper_node.destroy_node()

if __name__ == "__main__":
    pytest.main()

