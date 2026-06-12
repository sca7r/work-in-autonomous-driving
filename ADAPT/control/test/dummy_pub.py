import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DummyTwistPublisher(Node):
    def __init__(self):
        super().__init__('dummy_twist_publisher')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.publish_twist)

    def publish_twist(self):
        dummy = Twist()
        dummy.linear.x = 1.2
        dummy.angular.z = 0.5

        self.publisher.publish(dummy)
        self.get_logger().info('Published Twist: Linear Speed: %.2f, Angular Speed: %.2f' % (dummy.linear.x, dummy.angular.z))

def main(args=None):
    rclpy.init(args=args)

    dummy_twist_publisher = DummyTwistPublisher()

    rclpy.spin(dummy_twist_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dummy_twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
