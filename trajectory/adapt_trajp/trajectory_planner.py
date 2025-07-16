import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy.interpolate import CubicSpline
import math

class TrajectoryPlanner(Node):

    def __init__(self):
        super().__init__('trajectory_planner')


        """
        SUBSCRIBERS
        """  

        self.path_subscriber = self.create_subscription(PoseArray, '/route', self.path_callback, 1)
        self.drive_subscriber = self.create_subscription(Bool, '/drive', self.drive_callback, 10)
        self.park_for_subscriber = self.create_subscription(Bool, '/park', self.park_forward_callback, 1)
        self.park_rev_subscriber = self.create_subscription(Bool, '/park_reverse', self.park_reverse_callback, 1)
        self.yaw_subscriber = self.create_subscription(Vector3, '/euler_angles', self.yaw_callback, 10)
        self.loc_subscriber = self.create_subscription(PoseStamped, '/loc_pose', self.loc_callback, 10)
        self.subscription_spot = self.create_subscription(PoseStamped, '/selected_spot', self.spot_location_callback, 10)

        self.trajectory_visualize = self.create_publisher(Path,'/visualize', 1)
        self.forward_trajectory_visualize = self.create_publisher(Path, '/visualize_forward_park', 1)
        self.reverse_trajectory_visualize = self.create_publisher(Path, '/visualize_reverse_park', 1)

        """
        INITIALIZING VARIABLES WITH BOOLEAN OR NONE
        """     
        
        self.current_path = None
        self.selected_spot = None
        self.drive_state = None
        self.stop_drive_state = None
        self.park_state = None
        self.sent_drive_message = False
        self.sent_stop_drive_message = False
        self.sent_park_rev_message = False
        self.sent_park_for_message = False

        self.sent_driving_trajectory = False
        self.sent_parking_trajectory = False

    """
    LOCALIZATION AND ORIENTATION
    """     

        
    def loc_callback(self, msg):
        self.get_logger().info('Received current location')
        self.current_loc = msg

    def path_callback(self, msg):
        self.get_logger().info('Received current path')
        self.current_path = msg

    def yaw_callback(self, msg):
        self.get_logger().info(f'Received yaw angle: {msg.z}')
        self.current_yaw = msg.z

    def spot_location_callback(self, msg):
        self.selected_spot = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Received parking spot location: {self.selected_spot}')    

    """
    PUBLISHERS
    """    

    """
    THIS CALLBACK LISTENS TO BOOLEAN AND DECIDE TO PUBLISH THE DRIVE PATH OR TO SENT THE STOP MESSAGE
    """   

    #TRUE FOR DRIVE
    #FALSE FOR STOP  

    def drive_callback(self, msg):
        if msg is None:
            return

        self.get_logger().info(f'Received DRIVE state: {msg.data}')
        self.drive_state = msg.data

        if self.drive_state == True and self.sent_drive_message == False:
            self.trajectory_publisher = self.create_publisher(JointTrajectory, '/trajd',1)
            self.smooth_trajectory()
            self.sent_drive_message = True
            self.sent_stop_drive_message = False
         
        elif self.drive_state == False and self.sent_stop_drive_message == False:
            self.stop_trajectory_publisher = self.create_publisher(JointTrajectory, '/trajs', 1)
            self.stop_car()
            self.sent_stop_drive_message = True
            self.sent_drive_message = False

    """
    THIS CALLBACK LISTENS TO BOOLEAN AND DECIDE TO PUBLISH THE PARKING REVERSE FORWARD
    """

    def park_forward_callback(self, msg):

        if msg is None:
            return

        self.get_logger().info(f'Received PARK state: {msg.data}')
        self.park_state = msg.data

        if self.park_state == True and self.sent_park_for_message == False:
            self.generate_parking_trajectory()
            self.forward_trajectory_publisher = self.create_publisher(JointTrajectory, '/trajpf', 1)
            self.forward_park_path_publisher()   
            self.sent_park_for_message = True 

    """
    THIS CALLBACK LISTENS TO BOOLEAN AND DECIDE TO PUBLISH THE PARKING REVERSE PATH
    """          

    def park_reverse_callback(self, msg):
        if msg is None:
            return

        self.get_logger().info(f'Received PARK state: {msg.data}')
        self.park_state = msg.data

        if self.park_state == True and self.sent_park_rev_message == False:
            self.generate_parking_trajectory()
            self.reverse_trajectory_publisher = self.create_publisher(JointTrajectory, '/trajpr', 1)
            self.reverse_park_path_publisher()   
            self.sent_park_rev_message = True 

    """
    TRAJECTORY SMOOTHER TO INCREASE THE WAYPOINT COUNTS IN ORDER TO HAVE SMOOTH MOTION
    """          

    def smooth_trajectory(self):
        if self.current_path is None:
            return

        # Extract waypoints from the path message
        waypoints = np.array([[pose.position.x, pose.position.y] for pose in self.current_path.poses])

        if len(waypoints) < 3:
            self.get_logger().warn('Not enough waypoints to generate trajectory')
            return

        # Smooth the path using cubic spline interpolation
        x = waypoints[:, 0]
        y = waypoints[:, 1]
        distance = np.cumsum(np.sqrt(np.diff(x, prepend=x[0])**2 + np.diff(y, prepend=y[0])**2))
        cs_x = CubicSpline(distance, x)
        cs_y = CubicSpline(distance, y)
        dense_distance = np.linspace(distance[0], distance[-1], num=50)
        dense_x = cs_x(dense_distance)
        dense_y = cs_y(dense_distance)

        # Create and publish trajectory message
        trajectory_msg1 = JointTrajectory()
        trajectory_msg1.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg1.header.frame_id = 'map'
        trajectory_msg1.joint_names = ['x', 'y', 'velocity']

        # Set velocities to maximum speed
        velocities = np.full(dense_x.shape[0], 1.0)

        for i in range(len(dense_x)):
            point = JointTrajectoryPoint()
            point.positions = [dense_x[i], dense_y[i]]
            point.velocities = [velocities[i], velocities[i]]  
            trajectory_msg1.points.append(point)
            self.get_logger().info(f'Position: [{dense_x[i]}, {dense_y[i]}], Velocity: [{velocities[i]}]')
        
        
        self.trajectory_publisher.publish(trajectory_msg1) 

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for i in range(len(dense_distance)):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = dense_x[i]
            pose.pose.position.y = dense_y[i]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.trajectory_visualize.publish(path_msg)
        self.get_logger().info('Published visualized trajectory')

    """
    PARKING CALCULATION
    """        
    #two curvatures and three line segments are used to form the path
    
    def generate_parking_trajectory(self):
        if self.current_path is None and self.selected_spot is None:
            return

        # Define key points for parking trajectory
        theta = math.radians(30) 
        radius = 1.2
        initp = np.array([self.current_path.poses[-1].position.x, self.current_path.poses[-1].position.y])
        print(initp)

        interp = np.array([self.current_path.poses[-1].position.x, self.selected_spot[1]])
        print(interp)

        delta_theta = radius * math.tan(theta / 2)
        print("delta_theta:", delta_theta)

        interp1 = (interp[0],interp[1]-delta_theta)
        print("Intermediate position 1: ", interp1)

        interp2 = self.calculate_point_on_line(interp, math.tan(math.radians(60)), delta_theta)
        print("Intermediate position 2: ", interp2)

        delta_s = radius * (math.tan((math.pi/2 - theta)/2))
        print("delta_s:", delta_s)

        dist_s = delta_s - delta_theta
        print("dist s:", dist_s)

        crossp = self.calculate_point_on_line(interp, math.tan(math.radians(60)), delta_s)
        print("cross position: ", crossp)

        destp = (3.5 + delta_s, self.selected_spot[1])
        print("destination position: ", destp)

        goalp = self.selected_spot
        print("goal position: ", goalp)
       
        # Phase 1: InitP to InterP (straight line)
        phase1 = self.generate_straight_line(initp, interp1, 1.0)

        # Phase 2: InterP to TangentP (circle)
        phase2 = self.generate_circular_arc(interp1, interp2, radius, 1.0)

        # Phase 3: TangentP to CenterP (circle, reverse)
        phase3 = self.generate_straight_line(interp2, crossp, 1.0)

        # Phase 4: CenterP to GoalP (straight line, reverse)
        phase4 = self.generate_circular_arc(crossp, destp, radius, -1.0)

        phase5 = self.generate_straight_line(destp, goalp, -1.0)

        # Combine all phases
        self.trajectory_for = phase1 + phase2 + phase3 
        self.trajectory_rev = phase4 + phase5

    """
    FORWARD PATH PUBLISHER (FILLS THE MESSAGE)
    """        

    def forward_park_path_publisher(self):

        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = 'map'
        trajectory_msg.joint_names = ['x', 'y', 'velocity']

        for point in self.trajectory_for:
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.positions = point[:2]
            joint_trajectory_point.velocities = [point[2], point[2]]
            trajectory_msg.points.append(joint_trajectory_point)
            self.get_logger().info(f'Position: [{point[0]}, {point[1]}], Velocity: [{point[2]}]')

        self.forward_trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('publised forward parking trajectory')

        # Visualize forward parking trajectory
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in self.trajectory_for:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.forward_trajectory_visualize.publish(path_msg)
        self.get_logger().info('Published visualized forward parking trajectory')

    """
    REVERSE PATH PUBLISHER (FILLS THE MESSAGE)
    """    

    def reverse_park_path_publisher(self):

        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = 'map'
        trajectory_msg.joint_names = ['x', 'y', 'velocity']

        for point in self.trajectory_rev:
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.positions = point[:2]
            joint_trajectory_point.velocities = [point[2], point[2]]
            trajectory_msg.points.append(joint_trajectory_point)
            self.get_logger().info(f'Position: [{point[0]}, {point[1]}], Velocity: [{point[2]}]')

        self.reverse_trajectory_publisher.publish(trajectory_msg)  
        self.get_logger().info('publised reverse parking trajectory') 

        # Visualize reverse parking trajectory
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in self.trajectory_rev:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.reverse_trajectory_visualize.publish(path_msg)
        self.get_logger().info('Published visualized reverse parking trajectory')

    """
    HELPER FUNCTIONS
    """


    def generate_straight_line(self, start, end, linear_velocity):
        num_points = 10
        x = np.linspace(start[0], end[0], num_points)
        y = np.linspace(start[1], end[1], num_points)
        velocities = np.full(num_points, linear_velocity)
        return list(zip(x, y, velocities))

    def calculate_point_on_line(self, point_A, slope, distance):
    # Point A coordinates
        x_A, y_A = point_A
    
    # Calculate coefficients
        coef_x = 1 / math.sqrt(1 + slope**2)
        coef_y = slope / math.sqrt(1 + slope**2)
    
    # Calculate coordinates of the point on the line
        x_point = x_A - coef_x * distance
        y_point = y_A + coef_y * distance
    
        return (x_point, y_point)

    def generate_circular_arc(self, start, end, radius, linear_velocity):
        # Convert start and end points to numpy arrays
        start = np.array(start)
        end = np.array(end)

        # Calculate the midpoint between the start and end points
        mid_point = (start + end) / 2.0

        # Calculate the perpendicular bisector of the line segment connecting start and end points
        vec = end - start
        perp_vec = np.array([-vec[1], vec[0]])
        perp_vec = perp_vec / np.linalg.norm(perp_vec)

        # Calculate the distance from the midpoint to the center of the circle
        start_to_mid_dist = np.linalg.norm(mid_point - start)
        center_dist = np.sqrt(radius**2 - start_to_mid_dist**2)
        print(center_dist)

        # Calculate the possible centers of the circle
        center1 = mid_point + center_dist * perp_vec
        center2 = mid_point - center_dist * perp_vec

        # Determine the correct center (assuming the smaller angle is the correct one)
        if np.arctan2(start[1] - center1[1], start[0] - center1[0]) < np.arctan2(end[1] - center1[1], end[0] - center1[0]):
            center = center1
        else:
            center = center2

        # Calculate the angles for the start and end points with respect to the circle center
        angle_start = np.arctan2(start[1] - center[1], start[0] - center[0])
        angle_end = np.arctan2(end[1] - center[1], end[0] - center[0])

        # Generate points along the circular arc
        num_points = 10
        if angle_start > angle_end:
            angles = np.linspace(angle_start, angle_end + 2 * np.pi, num_points)
        else:
            angles = np.linspace(angle_start, angle_end, num_points)

        x = center[0] + radius * np.cos(angles)
        y = center[1] + radius * np.sin(angles)
        velocities = np.full(num_points, linear_velocity)

        return list(zip(x, y, velocities))
    
    """
    STOP FUNCTION
    THIS TAKES CARE TO FILL THE TRAJECTORY WITH 0 VALUES
    """
    
    def stop_car(self):
        # Stop the car logic
        # For simplicity, let's just publish a trajectory with all velocities set to 0
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = 'map'
        trajectory_msg.joint_names = ['x', 'y', 'velocity']

        stop_point = JointTrajectoryPoint()
        stop_point.positions = [0.0, 0.0]
        stop_point.velocities = [0.0, 0.0]
        self.get_logger().info(f'Position: [{stop_point.positions[0]}, {stop_point.positions[1]}], Velocity: [{stop_point.velocities[0]}]')

        trajectory_msg.points.append(stop_point)
        self.stop_trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('publised stop trajectory') 
    
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

