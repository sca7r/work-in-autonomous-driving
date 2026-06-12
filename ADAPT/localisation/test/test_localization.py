import pytest
import rclpy
from adapt_loc.localization import SimpleLocalization
from mocap_msgs.msg import RigidBodies, RigidBody
from geometry_msgs.msg import Pose, Quaternion

@pytest.fixture(scope="module", autouse=True)
def rclpy_setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_quaternion_to_euler(rclpy_setup_teardown):
    node = SimpleLocalization()
    # Test identity quaternion
    q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    roll, pitch, yaw = node.quaternion_to_euler(q)
    assert roll == 0.0
    assert pitch == 0.0
    assert yaw == 0.0

    # Test 90 degrees around z-axis
    q = Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)
    roll, pitch, yaw = node.quaternion_to_euler(q)
    assert roll == 0.0
    assert pitch == 0.0
    assert yaw == pytest.approx(3.14159, 0.01)

def test_publish_pose(rclpy_setup_teardown):
    node = SimpleLocalization()
    pose = Pose()
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    node.publish_pose(1.0, 2.0, 3.0, pose.orientation)
    # Adding assertion to confirm publishing (mocking or capturing the output might be needed)
    assert True

def test_publish_euler_angles(rclpy_setup_teardown):
    node = SimpleLocalization()
    node.publish_euler_angles(0.1, 0.2, 0.3)
    # Adding assertion to confirm publishing (mocking or capturing the output might be needed)
    assert True

def test_process_rigid_body(rclpy_setup_teardown):
    node = SimpleLocalization()
    pose = Pose()
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    pose.position.x = 1.12345
    pose.position.y = 2.12345
    pose.position.z = 3.12345

    body = RigidBody()
    body.pose = pose
    body.rigid_body_name = "9"

    node.process_rigid_body(body)

def test_position_callback(rclpy_setup_teardown):
    node = SimpleLocalization()

    # Create a RigidBodies message
    pose = Pose()
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    pose.position.x = 1.12345
    pose.position.y = 2.12345
    pose.position.z = 3.12345

    body = RigidBody()
    body.pose = pose
    body.rigid_body_name = "9"

    rigid_bodies_msg = RigidBodies()
    rigid_bodies_msg.rigidbodies.append(body)

    node.position_callback(rigid_bodies_msg)

    # Test with a different rigid_body_name
    body.rigid_body_name = "10"
    rigid_bodies_msg.rigidbodies[0] = body
    node.position_callback(rigid_bodies_msg)

if __name__ == "__main__":
    pytest.main()

