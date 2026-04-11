# echoで確認するので十分だったので使わないかも
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.get_logger().info("Subscribed to /joint_states")

    def listener_callback(self, msg):
        self.get_logger().info("Received JointState message:")
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            self.get_logger().info(
                f"  {name}: pos={pos:.3f}, vel={vel:.3f}, effort={eff:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
