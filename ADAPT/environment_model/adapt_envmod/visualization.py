import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3, Point, PoseStamped
from visualization_msgs.msg import Marker
from adapt_msgs.msg import VehData

class Environment(Node):
    """
    ROS 2 Node for managing and publishing vehicle poses and markers in a simulated environment.

    This node subscribes to vehicle pose and vehicle location topics. It publishes:
    - Transforms (`TransformStamped`) for visualizing the positions and orientations of vehicles.
    - Markers (`Marker`) for visual representation of vehicles in Rviz.
    
    It handles data from a adapt car and other cars publishing CAM in the environment.
    """
    def __init__(self):
        """
        Initialize the Environment node.

        Sets up subscriptions for vehicle pose and vehicle location data, and initializes
        publishers for vehicle transforms and markers. It also configures publishers for 
        specific car IDs (7, 10, and 9) for visualization purposes.
        """
        super().__init__('Environment')

        # Dictionaries to hold TF and Marker publishers for each car
        self.tf_publishers = {}
        self.marker_publishers = {}

        # Subscription to the localization pose topic for the location of our car
        self.loc_pose_subscription = self.create_subscription(PoseStamped, '/loc_pose', self.loc_pose_callback, 10)

        # Subscription to the vehicle location topic for other cars' data
        self.ev_location_subscription = self.create_subscription(VehData, '/ev_location', self.ev_location_callback, 10)

        # Log initialization message
        self.get_logger().info('Pose Transformer node initialized')

        # Initialize TF and Marker publishers for specific car IDs
        for car_id in [7, 10]:
            self.tf_publishers[car_id] = self.create_publisher(TransformStamped, f'/tf_{car_id}', 10)
            self.marker_publishers[car_id] = self.create_publisher(Marker, f'/car_marker_{car_id}', 10)

        # Additional publishers for car ID 9
        self.tf_publishers[9] = self.create_publisher(TransformStamped, '/tf_9', 10)
        self.marker_publishers[9] = self.create_publisher(Marker, '/car_marker_9', 10)

    def loc_pose_callback(self, msg):
        """
        Callback function for the localization pose subscription.

        Updates and publishes the transform and marker for the car with ID 9 based on
        the received PoseStamped message.

        Args:
            msg (PoseStamped): The PoseStamped message containing the localization data of the car.
        """
        car_id = 9
        data = msg.pose

        # Create and publish the TF message for car ID 9
        tf_msg = TransformStamped()
        tf_msg.header = msg.header
        tf_msg.child_frame_id = f"car_{car_id}"
        tf_msg.transform.translation.x = data.position.x
        tf_msg.transform.translation.y = data.position.y
        tf_msg.transform.translation.z = data.position.z
        tf_msg.transform.rotation = data.orientation
        tf_msg.header.frame_id = "map"

        self.tf_publishers[9].publish(tf_msg)
        self.get_logger().info(f'Published TF for car {car_id}: {tf_msg}')

        # Create and publish the Marker message for car ID 9
        marker_msg = Marker()
        marker_msg.header = msg.header
        marker_msg.ns = f"car_{car_id}"
        marker_msg.id = 9
        marker_msg.type = Marker.MESH_RESOURCE
        marker_msg.mesh_resource = "package://adapt_envmod/resource/adaptcar.obj"
        marker_msg.action = Marker.ADD
        marker_msg.pose = data
        marker_msg.scale = Vector3(x=0.1, y=0.1, z=0.1)
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.5
        marker_msg.color.g = 0.5
        marker_msg.color.b = 0.5

        self.marker_publishers[9].publish(marker_msg)
        self.get_logger().info(f'Published marker for car {car_id}: {marker_msg}')

    def ev_location_callback(self, msg):
        """
        Callback function for the vehicle location subscription.

        Updates and publishes the transform and marker for each car based on the received
        VehData message.

        Args:
            msg (VehData): The VehData message containing the position and orientation of a vehicle.
        """
        car_id = msg.id

        # Create and publish the TF message for the vehicle
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = f"car_{car_id}"
        tf_msg.transform.translation.x = msg.x
        tf_msg.transform.translation.y = msg.y
        tf_msg.transform.translation.z = msg.z
        tf_msg.transform.rotation.x = msg.x_rotation
        tf_msg.transform.rotation.y = msg.y_rotation
        tf_msg.transform.rotation.z = msg.z_rotation
        tf_msg.transform.rotation.w = msg.w_rotation

        self.tf_publishers[car_id].publish(tf_msg)
        self.get_logger().info(f'Published TF for car {car_id}: {tf_msg}')

        # Create and publish the Marker message for the vehicle
        marker_msg = Marker()
        marker_msg.header = tf_msg.header
        marker_msg.ns = f"car_{car_id}"
        marker_msg.id = car_id
        marker_msg.type = Marker.MESH_RESOURCE
        marker_msg.mesh_resource = "package://adapt_envmod/resource/adaptcar.obj"
        marker_msg.action = Marker.ADD
        marker_msg.pose.position = Point(x=tf_msg.transform.translation.x,
                                         y=tf_msg.transform.translation.y,
                                         z=tf_msg.transform.translation.z)
        marker_msg.pose.orientation.x = msg.x_rotation
        marker_msg.pose.orientation.y = msg.y_rotation
        marker_msg.pose.orientation.z = msg.z_rotation
        marker_msg.pose.orientation.w = msg.w_rotation
        marker_msg.scale = Vector3(x=0.1, y=0.1, z=0.1)

        # Set color based on car ID
        if car_id == 7:
            marker_msg.color.a = 1.0
            marker_msg.color.r = 1.0
            marker_msg.color.g = 1.0
            marker_msg.color.b = 0.0  # Yellow for car ID 7
        elif car_id == 10:
            marker_msg.color.a = 1.0
            marker_msg.color.r = 0.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 1.0  # Blue for car ID 10

        self.marker_publishers[car_id].publish(marker_msg)
        self.get_logger().info(f'Published marker for car {car_id}: {marker_msg}')

def main(args=None):
    """
    Main function to initialize and spin the Environment node.

    Initializes the ROS 2 Python client library, creates an instance of the Environment node,
    and starts the ROS event loop to process callbacks. Cleans up resources and shuts down
    the client library on exit.
    
    Args:
        args (list, optional): Command line arguments (default is None).
    """
    rclpy.init(args=args)
    environment = Environment()
    rclpy.spin(environment)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

