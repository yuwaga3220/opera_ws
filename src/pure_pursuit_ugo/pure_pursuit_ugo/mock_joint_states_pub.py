import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MockJointStatesPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mock_joint_states_publisher")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)

        self.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
        ]
        self.time_sec = 0.0
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.publish_mock_joint_states)

        self.get_logger().info("Publishing mock /joint_states at 20 Hz")

    def publish_mock_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        msg.position = [
            0.6 * math.sin(self.time_sec + phase)
            for phase in [0.0, 0.4, 0.8]
        ]
        msg.velocity = [
            0.6 * math.cos(self.time_sec + phase)
            for phase in [0.0, 0.4, 0.8]
        ]
        msg.effort = [0.0] * len(self.joint_names)

        self.publisher.publish(msg)
        self.time_sec += self.dt


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockJointStatesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
