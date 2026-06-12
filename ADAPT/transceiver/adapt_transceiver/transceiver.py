import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import String
from adapt_msgs.msg import VehData
import pymap3d as pm
import math
import numpy as np

#from vision_msgs.msg import Detection2DArray
from v2x.msg import CAM, ItsPduHeader, CoopAwareness, CamParameters, BasicContainer, ReferencePosition, Altitude
class Transceiver(Node):

    def __init__(self):
        super().__init__('transceiver_data')

        # Initializing the required variables and lists
        self.j = 0
        self.alt = 0
        self.lat = 0
        self.lon = 0
        self.send2 = []
        self.cam_data = []
        self.seen_ids = []
        self.east = 0.0
        self.north = 0.0
        self.up = 0.0 
        self.wgs84_yaw = 0.0
        
        #approximate reference values assigned    
        self.LAT0 =  50.24132213367954 
        self.LON0 = 11.321265180951718
        self.ALT0 = 0.0
        
        #create publishers for the components 
        #self.publisher_selectedspot_ = self.create_publisher(PoseStamped, '/selected_spot_location', 10)  # The Selected Spot Location.
        #self.selectedspot_callback()

        #this publisher will publish a message containing the info of ego vehicles
        self.publisher_evlocation_ = self.create_publisher(VehData, '/ev_location', 1)   # all Ego Vehicles Location.
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.evlocation_callback)

        #CAM_publisher of adapt vehicle with id no 9
        self.cam_publisher = self.create_publisher(CAM, '/cam_msgs', 1)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.cam_callback)
        self.i = 0


        # Subscriber Setup
        self.sub_localization = self.create_subscription(PoseStamped,'/loc_pose',self.localization_callback,10)        # The Ego Vehicle Location received.
        self.sub_localization_euler = self.create_subscription(Vector3,'/euler_angles',self.yaw_callback,10)  
        #self.sub_spotlocation = self.create_subscription(PoseStamped,'/selected_spot',self.spotlocation_callback,10)   # The Selected Spot Location received.    
        self.cam_global = self.create_subscription(CAM, '/cam_msgs', self.cam_subscriber, 1)                           # cam subscriptions of other remote vehicles
        #self.subscription_detection = self.create_subscription(Detection2DArray, '/detectnet/detections', self.detection_callback, 10) 


    def cam_callback(self): #CAM message

        #creating objects of the respective message fields and filling the overall message with required data
        cam_msg = CAM()
        pdu_header = ItsPduHeader()
        c_aware = CoopAwareness()
        c_para = CamParameters()
        bas_con = BasicContainer() 
        ref_pos = ReferencePosition() 
        c_alt = Altitude()

        c_alt.altitude_value = self.alt
        c_alt.altitude_confidence = 10
        
        ref_pos.latitude = self.lat
        ref_pos.longitude = self.lon
        ref_pos.altitude = c_alt

        bas_con.station_type = 5
        bas_con.reference_position = ref_pos

        c_para.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value = int(self.wgs84_yaw * 10)
        c_para.basic_container = bas_con
        c_aware.cam_parameters = c_para
        c_aware.generation_deltatime = 121  

        pdu_header.protocol_version = 2 
        pdu_header.message_id = 2 
        pdu_header.station_id = 9 

        cam_msg.cam = c_aware 
        cam_msg.header = pdu_header


        self.cam_publisher.publish(cam_msg)
        self.get_logger().info('My location is: %f, %f, %f' % (self.lat, self.lon, self.alt))
        self.get_logger().info('Yaw angle is: %f' % (self.wgs84_yaw))
        self.i += 1    

    #def selectedspot_callback(self):                           
        #msg = PoseStamped()  
        #self.publisher_selectedspot_.publish(msg)
        #self.get_logger().info ('Selected Parking Spot Location (%f, %f, %f)' % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

    def yaw_callback(self, msg: Vector3):

        self.yaw_angle = math.degrees(msg.z)   
        if self.yaw_angle < 0.0:
            self.yaw_angle += 360.0
            
            if 270.0 < self.yaw_angle <= 360.0:
                self.wgs84_yaw = (360 - self.yaw_angle) + 270
                self.get_logger().info('yaw angle (WGS84): %f' % (self.wgs84_yaw)) 
                return
            
            elif 180.0 <= self.yaw_angle <= 270.0:
                self.wgs84_yaw = (360 - self.yaw_angle) - 90  
                self.get_logger().info('yaw angle (WGS84): %f' % (self.wgs84_yaw))
                return  
            
        elif 0.0 <= self.yaw_angle <= 180.0:    
            self.wgs84_yaw = (180 - self.yaw_angle) + 90 
            self.get_logger().info('yaw angle (WGS84): %f' % (self.wgs84_yaw)) 
            return

    def evlocation_callback(self):                           # Callback for publishing selected spot location coordinate.
        #this publisher publish a message with location data of the ego vehicles 
        for info in self.cam_data:

            veh_msg = VehData()

            veh_msg.latitude = info['latitude'] 
            veh_msg.longitude = info['longitude']
            veh_msg.altitude = info['altitude']
            veh_msg.heading = info['heading']
            veh_msg.x_rotation = info['x_rotation']
            veh_msg.y_rotation = info['y_rotation']
            veh_msg.z_rotation = info['z_rotation']
            veh_msg.w_rotation = info['w_rotation']
            veh_msg.x = info['x']
            veh_msg.y = info['y']
            veh_msg.z = info['z']
            veh_msg.id = info['vehicle_id']

            self.publisher_evlocation_.publish(veh_msg)
            self.get_logger().info ('location of ev (id: %d)= latitude: %f, longitude: %f, altitude: %f, x: %f, y: %f,z: %f, heading: %f' % 
                                (veh_msg.id, veh_msg.latitude, veh_msg.longitude, veh_msg.altitude, veh_msg.x, veh_msg.y, veh_msg.z, veh_msg.heading)) 
            self.get_logger().info('rotation of vehicle: x: %f, y: %f, z: %f, w: %f' % (veh_msg.x_rotation, veh_msg.y_rotation, veh_msg.z_rotation, veh_msg.w_rotation))

        
    def localization_callback(self, msg): 
        
        self.pose_position = msg.pose.position
        self.pose_orientation = msg.pose.orientation
        
        #assign east, north and up coordinates
        self.e = msg.pose.position.x
        self.n = msg.pose.position.y
        self.u = msg.pose.position.z

        #convert enu coordinates into geodetic
        lat, lon, alt = pm.enu2geodetic(self.e, self.n, self.u, self.LAT0, self.LON0, self.ALT0)

        #latitude, longitude and altitude 
        self.lat = int(lat * 10000000)
        self.lon = int(lon * 10000000)
        self.alt = int(alt * 100)

        #self.get_logger().info('Got vehicle pose from /loc_pose')                       
    
    #def spotlocation_callback(self, msg: PoseStamped):  # Callback for Selected Spot Location data.
        #self.send2.append(msg)
        #self.get_logger().info('Selected parking spot (%f, %f, %f)' % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

    #def detection_callback(self, msg: Detection2DArray):
     #   for detection in msg.directions:
      #      self.update_grid(detection_bbox.center, 50)
       # self.get_logger().info('the detected objects ') 

    def cam_subscriber(self, msg):

        if msg.header.station_id == 9:
            return 

        # Get the vehicle ID from the message
        veh_id = msg.header.station_id

        self.latitude = msg.cam.cam_parameters.basic_container.reference_position.latitude * 1e-7 
        self.longitude = msg.cam.cam_parameters.basic_container.reference_position.longitude * 1e-7
        self.altitude = msg.cam.cam_parameters.basic_container.reference_position.altitude.altitude_value * 1e-2
        self.othyaw = float(msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value / 10)

        self.yaw_rad = math.radians(self.othyaw)
        self.x, self.y, self.z, self.w = self.quaternion_from_euler(self.yaw_rad)
        
        #self.get_logger().info('LAT0, LON0, ALT0=(%f, %f, %f)' % (self.LAT0, self.LON0, self.ALT0))
        #self.get_logger().info('LAT, LON, ALT=(%f, %f, %f)' % (self.latitude, self.longitude, self.altitude))
        
        East,North,Up = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.LAT0, self.LON0, self.ALT0)

        
        #self.get_logger().info('E, N, U=(%f, %f, %f)' % (East,North,Up))
       
        # Check if the vehicle ID is not in the list of seen IDs
        if veh_id not in self.seen_ids:
            # Add the vehicle ID to the list of seen IDs
            self.seen_ids.append(veh_id)
            # Create a dictionary for storing the data of the new vehicle
            vehicle_info = {
                'vehicle_id': veh_id,
                'latitude': self.latitude,
                'altitude':self.altitude,
                'longitude': self.longitude,
                'heading': self.othyaw, 
                'x_rotation': self.x,
                'y_rotation': self.y,
                'z_rotation': self.z,
                'w_rotation': self.w,
                'x': East,
                'y': North,
                'z': Up,
                'index': len(self.seen_ids)  # Assign the index based on the number of seen IDs
            }
            # Append the vehicle data to the cam_data list
            self.cam_data.append(vehicle_info)
        else:
            # If the vehicle ID is already in the list of seen IDs,
            # find the corresponding vehicle data and update it
            for vehicle_info in self.cam_data:
                if vehicle_info['vehicle_id'] == veh_id:
                    vehicle_info['latitude'] = self.latitude
                    vehicle_info['altitude'] = self.altitude
                    vehicle_info['longitude'] = self.longitude
                    vehicle_info['heading'] = self.othyaw
                    vehicle_info['x_rotation'] = self.x
                    vehicle_info['y_rotation'] = self.y
                    vehicle_info['z_rotation'] = self.z
                    vehicle_info['w_rotation'] = self.w
                    vehicle_info['x'] = East
                    vehicle_info['y'] = North
                    vehicle_info['z'] = Up
                    break

    def quaternion_from_euler(self, yaw):
        roll = 0.0
        pitch = 0.0
 
        x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        return [x, y, z, w]            
       

def main(args=None):
    rclpy.init(args=args)
 
    transceiver_data = Transceiver()

    rclpy.spin(transceiver_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    transceiver_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
