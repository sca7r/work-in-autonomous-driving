import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import argparse

class DummyStatePublisher(Node):

    def __init__(self, topic_name, state):
        super().__init__('dummy_state_publisher')
        self.publisher_ = self.create_publisher(Bool, topic_name, 1)
        timer_period =  1.0
        self.state = state
        self.timer = self.create_timer(timer_period, self.publish_state)

    def publish_state(self):
        msg = Bool()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing {msg.data} to {self.publisher_.topic}')

def main(args=None):
    parser = argparse.ArgumentParser(description='Dummy State Publisher for TrajectoryPlanner')
    parser.add_argument('--topic', type=str, required=True, help='The topic to publish to')
    parser.add_argument('--state', type=str, required=True, help='The state to publish (true or false)')
    args = parser.parse_args()

    # Convert state argument to boolean
    state = args.state.lower() in ('true', '1')

    rclpy.init(args=None)

    dummy_state_publisher = DummyStatePublisher(args.topic, state)

    rclpy.spin(dummy_state_publisher)

    dummy_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
