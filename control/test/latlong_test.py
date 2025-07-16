import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from adapt_latlongcon.latlong import LatLongController  

class IntegrationHelperNode(Node):
    def __init__(self):
        super().__init__('integration_helper_node')
        self.received_twist = None
        self.twist_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

    def twist_callback(self, msg):
        self.received_twist = msg

@pytest.fixture(scope="module", autouse=True)
def rclpy_setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

def create_and_publish_trajectory(controller_node):
    trajectory_msg = JointTrajectory()
    trajectory_msg.header.stamp = controller_node.get_clock().now().to_msg()
    trajectory_msg.header.frame_id = 'map'
    trajectory_msg.joint_names = ['x', 'y', 'velocity']

    # Generate example dense positions and velocities
    dense_x = np.linspace(0, 10, 100)  # Example dense x positions
    dense_y = np.zeros_like(dense_x)   # Example dense y positions
    velocities = np.full(dense_x.shape[0], 1.0)  # Example velocities

    for i in range(len(dense_x)):
        point = JointTrajectoryPoint()
        point.positions = [dense_x[i], dense_y[i]]
        point.velocities = [velocities[i], velocities[i]]
        trajectory_msg.points.append(point)
        controller_node.get_logger().info(f'Position: [{dense_x[i]}, {dense_y[i]}], Velocity: [{velocities[i]}]')

    controller_node.trajectory_publisher.publish(trajectory_msg)

def test_lat_long_controller_integration(rclpy_setup_teardown):
    controller_node = LatLongController()
    helper_node = IntegrationHelperNode()
    executor = SingleThreadedExecutor()
    executor.add_node(controller_node)
    executor.add_node(helper_node)

    try:
        # Simulate publishing a trajectory message
        create_and_publish_trajectory(controller_node)

        # Run the executor to process the messages
        executor.spin_once(timeout_sec=1)

        # Verify that the twist message was published
        assert helper_node.received_twist is not None
        assert helper_node.received_twist.linear.x != 0.0  #
        assert helper_node.received_twist.angular.z != 0.0  

    finally:
        executor.shutdown()
        controller_node.destroy_node()
        helper_node.destroy_node()

