import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import math

class SwingController(Node):
    def __init__(self):
        super().__init__('swing_controller')

        # 状態保持
        self.goal_direction = None
        self.is_swing_aligned = False

        # サブスクライバ
        self.create_subscription(Float64, '/goal_direction', self.goal_direction_callback, 10)
        self.create_subscription(Bool, '/is_swing_aligned', self.is_aligned_callback, 10)

        # パブリッシャ
        self.swing_pub = self.create_publisher(Float64, '/zx120/swing/cmd', 10)

        # タイマー実行
        self.create_timer(0.1, self.control_loop)

    def goal_direction_callback(self, msg: Float64):
        self.goal_direction = msg.data

    def is_aligned_callback(self, msg: Bool):
        self.is_swing_aligned = msg.data

    def control_loop(self):
        # データ未受信ならスキップ
        if self.goal_direction is None:
            self.get_logger().info("目標位置への角度を未受信です")
            return

        if not self.is_swing_aligned:
            # 指令としてgoal_directionを送信
            self.swing_pub.publish(Float64(data=self.goal_direction))
            self.get_logger().info(
                f"Not aligned. Publishing swing angle command: {math.degrees(self.goal_direction):.1f}°"
            )
        else:
            self.get_logger().info("Swing is already aligned. No command sent.")

def main(args=None):
    rclpy.init(args=args)
    node = SwingController()
    rclpy.spin(node)
    rclpy.shutdown()
