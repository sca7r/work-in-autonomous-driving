import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import numpy as np
from std_msgs.msg import Bool


class LatLongController(Node):

    def __init__(self):
        super().__init__('lat_long_controller')
        self.declare_parameters(
            namespace='',

        #declare look ahead distance as a parameter to adjust it as condition    
            parameters=[
                ('look_ahead_distance', 1.2)
            ]
        )
 
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value

        #wheel base of the car
        self.wheel_base = 0.5

        #twist publisher
        timer_period = 0.01
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(timer_period, self.twist_publisher)

        #Subscriptions

        self.path_sub_drive = self.create_subscription(JointTrajectory, '/trajd', self.path_for_drive, 10)
        self.path_sub_for = self.create_subscription(JointTrajectory, '/trajpf', self.path_for_park_for, 10)
        self.path_sub_rev = self.create_subscription(JointTrajectory, '/trajpr', self.path_for_park_rev, 10)
        self.stop_sub = self.create_subscription(Bool, '/stop', self.stop, 10)
        self.stop_drive_sub = self.create_subscription(JointTrajectory, '/stop', self.stop, 10)
        self.vehicle_position_sub = self.create_subscription(PoseStamped, '/loc_pose', self.vehicle_pose, 10)


        #return publishers
        #this return a boolean value to the behaviour planning and trajectory planning 
        #the park reverse topic triggers the reverse parking function in trajectory planning
        #the reached goal topic help the behaviour planning component understand the vehicle's status at the goal point  

        
        self.reverse = self.create_publisher(Bool, '/park_reverse', 10)
        self.reached_goal = self.create_publisher(Bool, '/reach_goal', 10)

        # Initialize variables with float, boolean and None
        self.current_pose = (0.0, 0.0)
        self.current_yaw = 0.0
        self.target_velocity = 0.0
        self.delta_deg = 0.0
        self.drive = False
        self.park_forward = False
        self.park_reverse = False
        self.e_stop = False
        self.path_drive = None
        self.path_park_for = None
        self.path_park_rev = None
        self.emergency_stop = None

#####################################################################################################################
    """ 
    TWIST EXECUTOR
    """         
    #there are four conditions
    #DRIVE - the cruising of the car from the user's location to the parking spot
    #FORWARD PARK - the actuation of the car in forward motion to align with the parking spot for the reverse state
    #REVERSE PARK - the actuation of the car in reverse motion to park successfully into the spot 

    def twist_publisher(self):
        if self.path_drive is not None and self.drive:

            """the publisher takes car of the drive from the initial position to the location near the parking spot"""

            twist_msg = Twist()
            twist_msg.linear.x = 0.5
            twist_msg.angular.z = float(self.pure_pursuit(self.path_drive))
            self.get_logger().info(f'Published Twist message for forward drive: Linear={twist_msg.linear.x}, Angular={twist_msg.angular.z}, position={self.current_pose}')
            self.cmd_vel.publish(twist_msg)

            #calculate the distance to last point in the path in order to know when to stop
            last_waypoint = self.path_drive.points[-1]
            x, y = self.current_pose
            distance_to_last_waypoint = math.sqrt((x - last_waypoint.positions[0])**2 + (y - last_waypoint.positions[1])**2)
            if distance_to_last_waypoint < 0.3:

                # Publish the boolean message indicating successful stop near the parking spot to the behaviour planning component
                self.reached_status = Bool()
                self.reached_status.data = True
                self.reached_goal.publish(self.reached_status)
                self.get_logger().info('The car is ready for park')

                self.drive = False
                self.path_drive = None 
                self.get_logger().warn('Stopped forward drive')

        elif self.path_park_for is not None and self.park_forward:

            """the publisher takes car of the drive from the location near the parking spot to a location
            where the car would allign itself so as to park successfully in reverse"""

            twist_msg = Twist()
            twist_msg.linear.x = 0.6
            twist_msg.angular.z = float(self.pure_pursuit(self.path_park_for))
            self.get_logger().info(f'Published Twist message for forward park: Linear={twist_msg.linear.x}, Angular={twist_msg.angular.z}')
            self.cmd_vel.publish(twist_msg)


            #calculate the range for the car to stop
            last_waypoint = self.path_park_for.points[-1]
            x, y = self.current_pose
            distance_to_last_waypoint = math.sqrt((x - last_waypoint.positions[0])**2 + (y - last_waypoint.positions[1])**2)
            if distance_to_last_waypoint < 0.3:

                # Publish the boolean message indicating successful allignment
                self.parking_status = Bool()
                self.parking_status.data = True
                self.reverse.publish(self.parking_status)
                self.get_logger().info('The car is ready for reverse motion')

                #set the drive flags to false and set the path to none
                #this is done to reset the drive function
                self.park_forward = False
                self.path_park_for = None 
                self.get_logger().info('Stopped forward parking.')

        elif self.path_park_rev is not None and self.park_reverse:
            twist_msg = Twist()

            """the publisher takes car of reverse motion of the car to drive into the parking spot"""
              
            self.get_logger().info('reverse parking initiated')
            twist_msg.linear.x = -0.8 
            twist_msg.angular.z = -24.62
            
            if (160.0 < self.current_yaw_deg < 180.0) or (-180.0 < self.current_yaw_deg < -170.0):
                twist_msg.linear.x = -0.8
                twist_msg.angular.z = 0.0
            self.get_logger().info(f'Published Twist message for reverse drive: Linear={twist_msg.linear.x}, Angular={twist_msg.angular.z}')
            self.cmd_vel.publish(twist_msg)
            
            last_waypoint = self.path_park_rev.points[-1]
            x, y = self.current_pose
            distance_to_last_waypoint = math.sqrt((x - last_waypoint.positions[0])**2 + (y - last_waypoint.positions[1])**2)
            if distance_to_last_waypoint < 0.2:

                #set the reverses flags to false and set the path to None in order to stop the car
                self.park_reverse = False
                self.path_park_rev = None 
                self.get_logger().info('Stopped reverse parking.')
            

        elif self.emergency_stop is not None and self.e_stop:

            """set the twist velocities to zero for the emergency stop"""
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_vel.publish(twist_msg)
            self.get_logger().info(f'Published Twist message for emergency stop: Linear={twist_msg.linear.x}, Angular={twist_msg.angular.z}')    
        else:

            """this is the default twist publisher so the car doesn't move when nothing is behing subscribed"""
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info(f'Published Twist message to stop: Linear={twist_msg.linear.x}, Angular={twist_msg.angular.z}')
            self.get_logger().warn('the car has stopped')
            self.cmd_vel.publish(twist_msg)
        
    def path_for_drive(self, msg):

        """this function store the drive message route and set the drive flag to true and set others to false"""
        self.path_drive = msg
        self.drive = True
        self.park_forward = False
        self.park_reverse = False
        self.e_stop = False

    def path_for_park_for(self, msg):

        """this function takes care to store the forward parking route and set the forward park flag to true and set others to false"""
        self.path_park_for = msg
        self.park_forward = True
        self.drive = False
        self.park_reverse = False
        self.e_stop = False            

    def path_for_park_rev(self, msg):

        """this function takes care to store the reverse parking route and set the reverse park flag to true and set others to false"""
        self.path_park_rev = msg
        self.target_point = (msg.points[9].positions[0], msg.points[9].positions[1])
        self.park_reverse = True
        self.drive = False
        self.park_forward = False
        self.e_stop = False


#####################################################################################################################
    """ 
    ORIENTATION AND LOCALIZATION
    """ 

    def vehicle_pose(self, msg):

        """Update current pose"""
        self.current_pose = (msg.pose.position.x, msg.pose.position.y)  
        self.get_logger().info(f'Current location received: {self.current_pose}')

        """Update current orientation"""
        self.orientation = msg.pose.orientation
        self.current_yaw = self.quaternion_to_euler(self.orientation)[2]
        self.current_yaw_deg = math.degrees(self.current_yaw)
        print(f'current orientation(deg):- {self.current_yaw_deg}')
        self.get_logger().info(f'Current orientation received: {self.current_yaw}')

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
    
 #####################################################################################################################
    """ 
    PURE PURSUIT CONTROLLER HELPER FUNCTIONS
    """  

    def find_nearest_waypoint(self, path):
        """
        Get closest idx to the vehicle
        """
        curr_xy = np.array(self.current_pose)
        waypoints = np.array([(point.positions[0], point.positions[1]) for point in path.points])
        distances = np.linalg.norm(waypoints - curr_xy, axis=1)
        nearest_idx = np.argmin(distances)
        self.get_logger().info(f'Nearest waypoint index: {nearest_idx}')
        print(nearest_idx)
        return nearest_idx

    def find_distance_index_based(self, idx, path):
        waypoint = path.points[idx]
        x1, y1 = waypoint.positions[0], waypoint.positions[1]
        distance = math.sqrt((x1 - self.current_pose[0]) ** 2 + (y1 - self.current_pose[1]) ** 2)
        return distance

    def idx_close_to_lookahead(self, idx, path):
        """
        Get closest index to lookahead that is greater than the lookahead
        """
        while idx < len(path.points) - 1 and self.find_distance_index_based(idx, path) < self.look_ahead_distance:
            idx += 1
        return idx
    
 #####################################################################################################################
    """ 
    PURE PURSUIT CONTROLLER 
    """


    def pure_pursuit(self, path):
        if not path.points:
            self.get_logger().warn('No waypoints received yet')
            return 0.0
        
        nearest_idx = self.find_nearest_waypoint(path)
        idx_near_lookahead = self.idx_close_to_lookahead(nearest_idx, path) 
        target_x = float(path.points[idx_near_lookahead].positions[0])
        target_y = float(path.points[idx_near_lookahead].positions[1])
        print(f'target point: {(target_x, target_y)}')

        """
        PURE PURSUIT CONTROLLER
        """

        # calculate alpha (angle between the goal point and the path point)
        x_delta = target_x - self.current_pose[0]
        y_delta = target_y - self.current_pose[1]
        alpha = np.arctan2(y_delta, x_delta) - self.current_yaw
        print(self.current_pose)

        # adjust alpha to be within -pi to pi range
        if alpha > np.pi:
            alpha -= 2 * np.pi
        if alpha < -np.pi:
            alpha += 2 * np.pi

        # Set the lookahead distance depending on the speed
        lookahead_distance = self.look_ahead_distance
        steering_angle = np.arctan2(2 * self.wheel_base * np.sin(alpha), lookahead_distance)

        # Convert steering angle to degrees
        steering_angle_deg = np.degrees(steering_angle)

        # Limit steering angle between -30 and 30 degrees
        steering_angle_deg = np.clip(steering_angle_deg, -30, 30)
        return float(steering_angle_deg)
        
 #####################################################################################################################
    """ 
    EMERGENCY STOP FUNCTION
    """    
    #It sets the emergency flag to true and other to false
    # it also publishes the twist with zero velocities    

    def stop(self, msg):
        if msg.data:
            # Stop the car logic
            self.emergency_stop = msg
         
            self.e_stop = True
            self.drive = False
            self.park_forward = False
            self.park_reverse = False
        
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_vel.publish(twist_msg)

            self.get_logger().info(f'Published Twist message to stop car: Linear={twist_msg.linear.x}, Angular={twist_msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    lat_long_controller = LatLongController()
    rclpy.spin(lat_long_controller)
    lat_long_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
