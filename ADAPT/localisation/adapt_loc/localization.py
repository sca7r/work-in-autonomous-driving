"""Localization node for ADAPT"""
import math
import rclpy
from rclpy.node import Node
from mocap_msgs.msg import RigidBodies # For OptiTrack
from geometry_msgs.msg import PoseStamped, Vector3

class SimpleLocalization(Node):
    """Localization node for processing and publishing pose data."""
    def __init__(self):
        super().__init__('localization')
        self.subscription = self.create_subscription(
            RigidBodies, '/pose_modelcars', self.position_callback, 10
            )
        self.pose_publisher = self.create_publisher(PoseStamped, '/loc_pose', 10)
        self.euler_publisher = self.create_publisher(Vector3, '/euler_angles', 10)
        self.get_logger().info("Localization node has started.")

    def position_callback(self, data):
        """Callback function for handling incoming RigidBodies data."""
        for body in data.rigidbodies:
            # Check for a specific rigid body which is the yellow model car- ROS2 ID = 9
            if body.rigid_body_name == "9":
                self.process_rigid_body(body)

    def process_rigid_body(self, body):
        """Process the rigid body data and publish pose and euler angles."""
        # Convert quaternion to Euler angles (returned in radians)
        roll, pitch, yaw = self.quaternion_to_euler(body.pose.orientation)

        # Round position coordinates to 4 decimal places for precision
        rounded_x = round(body.pose.position.x, 4)
        rounded_y = round(body.pose.position.y, 4)
        rounded_z = round(body.pose.position.z, 4)

        # Create and publish PoseStamped and Vector3 messages
        self.publish_pose(rounded_x, rounded_y, rounded_z, body.pose.orientation)
        self.publish_euler_angles(roll, pitch, yaw)

    def quaternion_to_euler(self, orientation):
        """Convert quaternion to Euler angles."""
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w

        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def publish_pose(self, x, y, z, orientation):
        """Publish the pose as a PoseStamped message in ENU Coordinates."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link_7'
        pose_msg.pose.position.x = x  # meters
        pose_msg.pose.position.y = y  # meters
        pose_msg.pose.position.z = z  # meters
        pose_msg.pose.orientation = orientation
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(
            f'Published PoseStamped: Position - x: {x} m, y: {y} m, z: {z} m'
            )

    def publish_euler_angles(self, roll, pitch, yaw):
        """Publish the Euler angles as a Vector3 message."""
        euler_msg = Vector3()
        euler_msg.x = roll   # radians
        euler_msg.y = pitch   # radians
        euler_msg.z = yaw   # radians
        self.euler_publisher.publish(euler_msg)
        self.get_logger().info(
            f'Published Euler Angles: Roll: {roll} rad, Pitch: {pitch} rad, Yaw: {yaw} rad'
            )


def main(args=None):
    """Main function to initialize and spin the localization node."""
    rclpy.init(args=args)
    localization_node = SimpleLocalization()
    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
