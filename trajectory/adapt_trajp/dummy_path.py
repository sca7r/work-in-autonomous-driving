import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.path_publisher = self.create_publisher(PoseArray, '/route', 1)
        self.publish_path()

    def publish_path(self):
        path_msg = PoseArray()
        path_msg.header.frame_id = 'map'

        points = [
            (0.5, 0.7),
            (1.0, 0.7),
            (1.5, 0.7),
            (2.0, 0.7),
            (2.5, 0.7),
            (3.0, 0.7),
            (3.25, 1.0),
            (3.5, 1.4),
            (3.5, 2.0),
            (3.5, 2.4),
            (3.5, 3.0),
            (3.5, 3.4),
            (3.5, 4.0),
            (3.5, 4.5),
        ]

        for point in points:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        for idx, pose in enumerate(path_msg.poses):
            self.get_logger().info(f'Point {idx}: x = {pose.position.x}, y = {pose.position.y}')
            
        self.path_publisher.publish(path_msg)
        self.get_logger().info('Path published')

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
