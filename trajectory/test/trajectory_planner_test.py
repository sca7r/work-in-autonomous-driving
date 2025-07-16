import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseArray, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class IntegrationHelperNode(Node):
    def __init__(self):
        super().__init__('integration_helper_node')
        self.received_path = None
        self.path_subscription = self.create_subscription(
            PoseArray,
            '/route',
            self.path_callback,
            10
        )

    def path_callback(self, msg):
        self.received_path = msg

@pytest.fixture(scope="module", autouse=True)
def rclpy_setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_trajectory_planner_integration(rclpy_setup_teardown):
    from adapt_trajp.trajectory_planner import TrajectoryPlanner

    trajectory_planner_node = TrajectoryPlanner()
    helper_node = IntegrationHelperNode()
    executor = SingleThreadedExecutor()
    executor.add_node(trajectory_planner_node)
    executor.add_node(helper_node)

    try:
        # Simulate receiving a PoseArray message for the path
        path_msg = PoseArray()
        poses = [Pose(), Pose(), Pose()]
        poses[0].position.x, poses[0].position.y = 0.0, 0.0
        poses[1].position.x, poses[1].position.y = 1.0, 1.0
        poses[2].position.x, poses[2].position.y = 2.0, 2.0
        path_msg.poses.extend(poses)
        trajectory_planner_node.path_callback(path_msg)

        # Simulate receiving a JointTrajectory message to start driving
        trajectory_msg = JointTrajectory()
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [1.0, 1.0]
        trajectory_point.velocities = [1.0, 1.0]
        trajectory_msg.points.append(trajectory_point)
        trajectory_planner_node.drive_callback(trajectory_msg)

        trajectory_msg = JointTrajectory()
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [2.0, 2.0]
        trajectory_point.velocities = [1.0, 1.0]
        trajectory_msg.points.append(trajectory_point)
        trajectory_planner_node.stop_drive_callback(trajectory_msg)

        trajectory_msg = JointTrajectory()
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [2.0, 2.0]
        trajectory_point.velocities = [1.0, 1.0]
        trajectory_msg.points.append(trajectory_point)
        trajectory_planner_node.stop_park_callback(trajectory_msg)

        trajectory_msg = JointTrajectory()
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [2.0, 2.0]
        trajectory_point.velocities = [1.0, 1.0]
        trajectory_msg.points.append(trajectory_point)
        trajectory_planner_node.park_forward_callback(trajectory_msg)

        trajectory_msg = JointTrajectory()
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [2.0, 2.0]
        trajectory_point.velocities = [1.0, 1.0]
        trajectory_msg.points.append(trajectory_point)
        trajectory_planner_node.park_reverse_callback(trajectory_msg)

        trajectory_msg = JointTrajectory()
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [2.0, 2.0]
        trajectory_point.velocities = [1.0, 1.0]
        trajectory_msg.points.append(trajectory_point)
        trajectory_planner_node.emergency_stop_callback(trajectory_msg)

        # Run the executor to process the messages
        executor.spin_once(timeout_sec=1)

        # Verify that the Path message was published
        assert helper_node.received_path is not None
        assert len(helper_node.received_path.poses) > 0
        for pose in helper_node.received_path.poses:
            assert pose.position.x != 0.0 or pose.position.y != 0.0

    finally:
        executor.shutdown()
        trajectory_planner_node.destroy_node()
        helper_node.destroy_node()
