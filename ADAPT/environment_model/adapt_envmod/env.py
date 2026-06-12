import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped
from adapt_msgs.msg import DetectedObjects, DetectedObject
from std_msgs.msg import Bool
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class EnvModel(Node):
    def __init__(self):
        """
        Initialize the EnvModel node.

        Sets up the subscriptions to LaserScan and PoseStamped topics, initializes the 
        transform broadcaster and listener, and sets up publishers for detected objects and 
        stop messages. Initializes a timer for periodic processing. Declares parameters for 
        angle start and stop range.
        """
        super().__init__('env_model')

        # Declare parameters with default values
        self.declare_parameter('angle_start', 1.0472)  # Default angle offset for scan correction
        self.declare_parameter('stop_range', 0.95)     # Default stop range in meters

        # Retrieve parameters
        self.angle_start = self.get_parameter('angle_start').get_parameter_value().double_value
        self.stop_range = self.get_parameter('stop_range').get_parameter_value().double_value

        # Set up QoS profile for subscriptions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        # For critical messages
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize last_scan_msg to avoid AttributeError
        self.last_scan_msg = None

        # Subscribe to the LaserScan topic
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)

        # Subscribe to the PoseStamped topic
        self.pose_subscription = self.create_subscription(PoseStamped, '/loc_pose', self.pose_callback, qos_profile)

        # Initialize the transform broadcaster and publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.detected_objects_publisher = self.create_publisher(DetectedObjects, '/scans', 10)
        self.stop_publisher = self.create_publisher(Bool, '/stop', reliable_qos)
        self.base_link_pose = None  # Initialize the base link pose

        # Initialize the transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically process and publish data
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("EnvModel Node initialized")

    def pose_callback(self, pose_msg):
        """
        Callback function for pose subscription.

        Updates the base link pose with the received PoseStamped message.

        Args:
            pose_msg (PoseStamped): The PoseStamped message containing the new pose information.
        """
        self.base_link_pose = pose_msg.pose
        self.get_logger().debug("Updated base link pose")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.

        This function converts roll, pitch, and yaw angles to a quaternion representation.
        The quaternion is used for rotation transformations in 3D space.

        Args:
            roll (float): The roll angle in radians.
            pitch (float): The pitch angle in radians.
            yaw (float): The yaw angle in radians.

        Returns:
            list: A list containing the quaternion components [qx, qy, qz, qw].
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles.

        This function converts a quaternion (x, y, z, w) to roll, pitch, and yaw angles.

        Args:
            x (float): The x component of the quaternion.
            y (float): The y component of the quaternion.
            z (float): The z component of the quaternion.
            w (float): The w component of the quaternion.

        Returns:
            tuple: A tuple containing the roll, pitch, and yaw angles in radians.
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def scan_callback(self, scan_msg):
        """
        Callback function for laser scan subscription.

        Processes the received LaserScan message and stores it for later use.

        Args:
            scan_msg (LaserScan): The LaserScan message containing distance readings.
        """
        self.get_logger().debug(f"Received a scan message with {len(scan_msg.ranges)} ranges")
        self.last_scan_msg = scan_msg  # Store the latest scan message

    def timer_callback(self):
        """
        Timer callback function to process and publish data periodically.

        This function processes the latest LaserScan message to detect objects, transforms 
        their coordinates from the laser frame to the base link frame, and publishes the 
        detected objects. It also checks if any objects are within the stop range and 
        publishes a stop message accordingly.

        - Processes each range from the LaserScan message.
        - Filters and transforms coordinates from the laser frame to the base link frame.
        - Logs details of objects within the stop range.
        - Publishes detected objects and a stop message.
        """
        if self.last_scan_msg is None:
            self.get_logger().warn("No scan data available to process")
            return

        detected_objects_msg = DetectedObjects()
        detected_objects_msg.header = self.last_scan_msg.header
        detected_objects_msg.header.frame_id = '9/base_link'

        obstacle_detected = False
        objects_in_stop_range = []

        for i in range(len(self.last_scan_msg.ranges)):
            angle = self.angle_start + self.last_scan_msg.angle_min + i * self.last_scan_msg.angle_increment

            # Filter by angle range
            if -0.5 < angle < 0.5:
                if self.last_scan_msg.ranges[i] != float('inf') and self.last_scan_msg.ranges[i] != float('nan') and self.last_scan_msg.ranges[i] != 0.0:
                    distance = self.last_scan_msg.ranges[i]
                    x_laser = distance * math.cos(angle)
                    y_laser = distance * math.sin(angle)

                    # Transform the (x, y) coordinates to the '9/base_link' frame
                    try:
                        transform = self.tf_buffer.lookup_transform('9/base_link', '9/laser_frame', rclpy.time.Time())
                        x_base, y_base = self.transform_coordinates_to_base_link(x_laser, y_laser, transform)
                    except Exception as e:
                        self.get_logger().warn(f"Transform lookup failed: {str(e)}")
                        continue

                    transformed_distance = math.sqrt(x_base**2 + y_base**2)
                    transformed_angle = math.atan2(y_base, x_base)

                    detected_object = DetectedObject()
                    detected_object.distance = transformed_distance
                    detected_object.angle = transformed_angle
                    detected_objects_msg.objects.append(detected_object)

                    # Log detected objects
                    if transformed_distance <= self.stop_range:
                        objects_in_stop_range.append(detected_object)
                        self.get_logger().debug(f"Detected object within stop range with distance: {transformed_distance:.2f} meters and angle: {transformed_angle:.2f} radians")

        # Publish detected objects
        self.detected_objects_publisher.publish(detected_objects_msg)

        # Check for obstacles within the stop range
        obstacle_detected = any(obj.distance <= self.stop_range for obj in detected_objects_msg.objects)

        # Publish stop message
        stop_msg = Bool()
        stop_msg.data = obstacle_detected
        self.stop_publisher.publish(stop_msg)

        if obstacle_detected:
            self.get_logger().info("Obstacle detected! Publishing STOP message.")
        else:
            self.get_logger().info("No obstacles detected. Publishing CONTINUE message.")

    def transform_coordinates_to_base_link(self, x, y, transform):
        """
        Transform coordinates from the laser frame to the base link frame.

        Applies the given transform (which includes translation and rotation) to the (x, y) 
        coordinates in the laser frame to obtain coordinates in the base link frame.

        Args:
            x (float): The x coordinate in the laser frame.
            y (float): The y coordinate in the laser frame.
            transform (TransformStamped): The transform from the laser frame to the base link frame.

        Returns:
            tuple: A tuple containing the transformed x and y coordinates in the base link frame.
        """
        # Extract the translation and rotation from the transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(rotation.x, rotation.y, rotation.z, rotation.w)

        # Apply the translation and rotation to the coordinates
        x_base = (x * math.cos(yaw) - y * math.sin(yaw)) + translation.x
        y_base = (x * math.sin(yaw) + y * math.cos(yaw)) + translation.y

        return x_base, y_base

def main():
    """
    Main function to initialize and spin the EnvModel node.

    Initializes the ROS 2 Python client library, creates an instance of the EnvModel node, 
    sets its logging level, and enters the ROS event loop to process callbacks. Cleans up 
    resources and shuts down the client library on exit.
    """
    rclpy.init()
    node = EnvModel()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

