import rclpy
from rclpy.node import Node
from v2x.msg import CPM, PerceivedObject, ObjectClassWithConfidence
from vision_msgs.msg import Detection2DArray

object_list_dict = {
    1: "Background",
    2: "Car",
    3: "Traffic Light",
    4: "Potted Plant",
    5: "Person",
}

class CpmPublisher(Node):
    def __init__(self):
        super().__init__('cpm_publisher')
        self.subscription = self.create_subscription(Detection2DArray, '/detectnet/detections', self.detections_callback, 10)
        self.publisher = self.create_publisher(CPM, '/detected_objects', 10)

    def detections_callback(self, msg):
        cpm_msg = CPM()

        cpm_msg.header.message_id = 14

        cpm_msg.payload.management_container.segmentation_info.total_msg_no = 1
        cpm_msg.payload.management_container.segmentation_info.this_msg_no = 1

        cpm_msg.payload.cpm_containers.originating_vehicle_container_is_present = True
        cpm_msg.payload.cpm_containers.originating_vehicle_container.vehicle_orientation_angle.wgs84_angle_value = 3601
        cpm_msg.payload.cpm_containers.originating_vehicle_container.vehicle_orientation_angle.angle_confidence = 127
        cpm_msg.payload.cpm_containers.originating_vehicle_container.pitch_angle.cartesian_angle_value = 3601
        cpm_msg.payload.cpm_containers.originating_vehicle_container.pitch_angle.angle_confidence = 127
        cpm_msg.payload.cpm_containers.originating_vehicle_container.roll_angle.cartesian_angle_value = 3601
        cpm_msg.payload.cpm_containers.originating_vehicle_container.roll_angle.angle_confidence = 127

        cpm_msg.payload.cpm_containers.originating_rsu_container_is_present = False

        cpm_msg.payload.cpm_containers.perceived_object_container.number_of_perceived_objects = len(msg.detections)

        perceived_objects = []

        for i in range(0, len(msg.detections)):
            perceived_object = PerceivedObject()

            perceived_object.object_id = i
            
            # List of all classification results for one object
            classifications = []
            
            # Iterate over all classification results
            for j in range(0, len(msg.detections[i].results)):
                classification = ObjectClassWithConfidence()
                
                # Convert detectnet object id to cpm object id
                object_class_id = ord(msg.detections[i].results[j].id) + 1

                # Get class name from the object_list_dict
                class_name = object_list_dict.get(object_class_id, "Unknown")

                # Log detected class ID and name
                self.get_logger().info("Object {}: Detected class: {} - {}".format(i + 1, object_class_id, class_name))

                # Check if object id is known
                if object_class_id in object_list_dict:
                    classification.object_class.vehicle_sub_class = object_class_id
                else:
                    classification.object_class.vehicle_sub_class = 0  # unknown

                # Detection confidence from detectnet detections
                classification.confidence = int((msg.detections[i].results[j].score * 100))
                classifications.append(classification)

            perceived_object.classification = classifications
            perceived_objects.append(perceived_object)

        # Set perceived objects of cpm message
        cpm_msg.payload.cpm_containers.perceived_object_container.perceived_objects = perceived_objects

        # Publish the cpm messages
        self.publisher.publish(cpm_msg)


def main(args=None):
    rclpy.init(args=args)
    cpm_publisher = CpmPublisher()
    rclpy.spin(cpm_publisher)
    cpm_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

