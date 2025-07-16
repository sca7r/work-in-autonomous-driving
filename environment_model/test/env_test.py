import sys
import os
import unittest
import rclpy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from unittest.mock import MagicMock

# Add the parent directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from adapt_envmod.env import EnvModel  

class TestEnvModel(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.env_model = EnvModel()
        self.env_model.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.tf_broadcaster = TransformBroadcaster(self.env_model)
        
        # Mock the publishers
        self.env_model.detected_objects_publisher = MagicMock()
        self.env_model.stop_publisher = MagicMock()

    def tearDown(self):
        self.env_model.destroy_node()
        rclpy.shutdown()

    # TC_01: test if the Node can be created
    def test_init_node(self):
        self.assertIsInstance(self.env_model, EnvModel)
        self.assertEqual(self.env_model.get_name(), 'env_model')

    # TC_02: test if the pose_callback is working properly
    def test_pose_callback(self):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 2.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.env_model.pose_callback(pose_msg)

        self.assertIsNotNone(self.env_model.base_link_pose)
        self.assertEqual(self.env_model.base_link_pose.position.x, 1.0)
        self.assertEqual(self.env_model.base_link_pose.position.y, 2.0)
        self.assertEqual(self.env_model.base_link_pose.position.z, 0.0)

    # TC_03: test if the scan_callback is working properly
    def test_scan_callback(self):
        laserscan_msg = LaserScan()
        laserscan_msg.ranges = [0.5, 1.5, float('inf'), 0.75]
        laserscan_msg.angle_min = -math.pi / 4
        laserscan_msg.angle_max = math.pi / 4
        laserscan_msg.angle_increment = math.pi / 8

        # Create a transform with valid frame IDs
        transform = TransformStamped()
        transform.header.stamp = rclpy.time.Time().to_msg()
        transform.header.frame_id = '9/base_link'
        transform.child_frame_id = '9/laser_frame'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.env_model.tf_buffer.set_transform(transform, 'default_authority')

        self.env_model.base_link_pose = Pose()
        self.env_model.base_link_pose.position.x = 0.0
        self.env_model.base_link_pose.position.y = 0.0
        self.env_model.base_link_pose.position.z = 0.0
        self.env_model.base_link_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.env_model.scan_callback(laserscan_msg)

        # Check that the detected objects message was published
        self.env_model.detected_objects_publisher.publish.assert_called()
        # Check the content of the published message if needed
        detected_objects_msg = self.env_model.detected_objects_publisher.publish.call_args[0][0]
        self.assertEqual(len(detected_objects_msg.objects), 1)

    # TC_04: test if object detection within stop range triggers stop signal
    def test_object_detection_within_stop_range(self):
        laserscan_msg = LaserScan()
        laserscan_msg.ranges = [0.5, 0.9, float('inf'), 0.75]
        laserscan_msg.angle_min = -math.pi / 4
        laserscan_msg.angle_max = math.pi / 4
        laserscan_msg.angle_increment = math.pi / 8

        # Create a transform with valid frame IDs
        transform = TransformStamped()
        transform.header.stamp = rclpy.time.Time().to_msg()
        transform.header.frame_id = '9/base_link'
        transform.child_frame_id = '9/laser_frame'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.env_model.tf_buffer.set_transform(transform, 'default_authority')

        self.env_model.base_link_pose = Pose()
        self.env_model.base_link_pose.position.x = 0.0
        self.env_model.base_link_pose.position.y = 0.0
        self.env_model.base_link_pose.position.z = 0.0
        self.env_model.base_link_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.env_model.scan_callback(laserscan_msg)

        # Check that the stop message was published
        self.env_model.stop_publisher.publish.assert_called()
        # Check the content of the published message if needed
        stop_msg = self.env_model.stop_publisher.publish.call_args[0][0]
        self.assertTrue(stop_msg.data)

if __name__ == '__main__':
    unittest.main()

