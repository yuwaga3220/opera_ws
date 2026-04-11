import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # 状態変数
        self.latest_twist = Twist()
        self.should_stop = False

        # subscribe：ナビゲーション速度
        self.sub_nav = self.create_subscription(
            Twist,
            '/nav_cmd_vel',
            self.nav_callback,
            10
        )

        # subscribe：障害物停止フラグ
        self.sub_stop = self.create_subscription(
            Bool,
            '/obstacle_stop',
            self.stop_callback,
            10
        )

        # publish：速度トピック
        self.pub = self.create_publisher(
            Twist,
            '/zx120/tracks/cmd_vel',
            10
        )

        self.get_logger().info("cmd_vel_mux is running...")

    def nav_callback(self, msg):
        self.latest_twist = msg
        self.publish_muxed()

    def stop_callback(self, msg):
        self.should_stop = msg.data
        self.get_logger().info(f"Obstacle flag updated: {self.should_stop}")
        self.publish_muxed()

    def publish_muxed(self):
        twist = Twist()
        if not self.should_stop:
            twist = self.latest_twist
        else:
            self.get_logger().warn("Obstacle detected: Publishing stop twist.")
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
