import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Point
from visualization_msgs.msg import Marker
from adapt_msgs.msg import VehData
from adapt_envmod.visualization import Environment

import tf2_ros
import tf2_geometry_msgs

class MockNode(Node):
    def __init__(self):
        super().__init__('mock_node')

class TestEnvironment(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize ROS client library
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown ROS client library
        rclpy.shutdown()

    def setUp(self):
        self.node = MockNode()
        self.environment = Environment(node=self.node)

    def tearDown(self):
        self.environment.destroy_node()

    def test_node_initialization(self):
        self.assertIsNotNone(self.environment)
        self.assertIsInstance(self.environment, Environment)
        self.assertEqual(self.environment.get_name(), 'Environment')

    def test_loc_pose_callback(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 3.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        # Simulate callback
        self.environment.loc_pose_callback(pose_msg)

        # Verify TF message using tf2_ros.Buffer and TransformListener
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, self.node)
        transform = buffer.lookup_transform('map', 'car_9', rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))

        # Assert transform values
        self.assertAlmostEqual(transform.transform.translation.x, 1.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.translation.y, 3.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.translation.z, 0.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.rotation.x, 0.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.rotation.y, 0.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.rotation.z, 0.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.rotation.w, 1.0, delta=0.01)

    def test_ev_location_callback(self):
        veh_data_msg = VehData()
        veh_data_msg.id = 7
        veh_data_msg.x = 2.0
        veh_data_msg.y = 4.0
        veh_data_msg.z = 0.0
        veh_data_msg.x_rotation = 0.0
        veh_data_msg.y_rotation = 0.0
        veh_data_msg.z_rotation = 0.0
        veh_data_msg.w_rotation = 1.0

        # Simulate callback
        self.environment.ev_location_callback(veh_data_msg)

        # Verify TF message using tf2_ros.Buffer and TransformListener
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, self.node)
        transform = buffer.lookup_transform('map', f'car_{veh_data_msg.id}', rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))

        # Assert transform values
        self.assertAlmostEqual(transform.transform.translation.x, 2.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.translation.y, 4.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.translation.z, 0.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.rotation.x, 0.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.rotation.y, 0.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.rotation.z, 0.0, delta=0.01)
        self.assertAlmostEqual(transform.transform.rotation.w, 1.0, delta=0.01)

    def test_marker_publishing(self):
        veh_data_msg = VehData()
        veh_data_msg.id = 7
        veh_data_msg.x = 2.0
        veh_data_msg.y = 4.0
        veh_data_msg.z = 0.0
        veh_data_msg.x_rotation = 0.0
        veh_data_msg.y_rotation = 0.0
        veh_data_msg.z_rotation = 0.0
        veh_data_msg.w_rotation = 1.0

        # Simulate callback
        self.environment.ev_location_callback(veh_data_msg)

        # Check published marker message
        marker_msg = self.node.create_subscription(Marker, f'/car_marker_{veh_data_msg.id}', self.marker_callback, 10)

    def marker_callback(self, marker_msg):
        # Verify the contents of the marker message
        self.assertEqual(marker_msg.ns, f'car_{veh_data_msg.id}')
        self.assertEqual(marker_msg.id, veh_data_msg.id)
        self.assertEqual(marker_msg.type, Marker.MESH_RESOURCE)
        self.assertEqual(marker_msg.mesh_resource, "file:///home/af/adapt_main/src/adapt_envmod/Car/adaptcar.obj")
        self.assertEqual(marker_msg.action, Marker.ADD)
        self.assertAlmostEqual(marker_msg.pose.position.x, 2.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.pose.position.y, 4.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.pose.position.z, 0.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.pose.orientation.x, 0.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.pose.orientation.y, 0.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.pose.orientation.z, 0.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.pose.orientation.w, 1.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.scale.x, 0.1, delta=0.01)
        self.assertAlmostEqual(marker_msg.scale.y, 0.1, delta=0.01)
        self.assertAlmostEqual(marker_msg.scale.z, 0.1, delta=0.01)
        self.assertAlmostEqual(marker_msg.color.a, 1.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.color.r, 1.0 if veh_data_msg.id == 7 else 0.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.color.g, 1.0 if veh_data_msg.id == 7 else 0.0, delta=0.01)
        self.assertAlmostEqual(marker_msg.color.b, 0.0 if veh_data_msg.id == 7 else 1.0, delta=0.01)

if __name__ == '__main__':
    unittest.main()


