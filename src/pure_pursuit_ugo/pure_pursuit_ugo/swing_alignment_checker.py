import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

class SwingAlignmentChecker(Node):
    def __init__(self):
        super().__init__('swing_alignment_checker')

        # 状態保持
        self.swing_joint = None
        self.goal_position = None
        self.angle_tolerance_deg = 2.0  # 許容範囲（度）

        # サブスクライバ
        self.create_subscription(JointState, '/zx120/joint_states', self.swing_joint_callback, 10) # 現在のswing角を取得
        self.create_subscription(Point, '/goal_position', self.goal_callback, 10) # 目標位置座標を取得

        # パブリッシャ
        self.pub = self.create_publisher(Bool, '/is_swing_aligned', 10)
        self.goal_direction_pub = self.create_publisher(Float64, '/goal_direction', 10)

        # タイマー
        self.create_timer(0.1, self.check_alignment)

    def swing_joint_callback(self, msg: JointState):
        if len(msg.position) > 0:
            self.swing_joint = msg.position[0]

    def goal_callback(self, msg: Point):
        self.goal_position = msg

    def check_alignment(self):
        if self.swing_joint is None or self.goal_position is None:
            self.get_logger().info("現在のswing角度または目標位置が未取得です")
            return

        # 目標方向ベクトル
        dx = self.goal_position.x
        dy = self.goal_position.y

        # ゴール方向の角度（rad）
        goal_direction = math.atan2(dy, dx)

        # 差分角（-π〜π に正規化）
        diff_rad = math.atan2(math.sin(goal_direction - self.swing_joint), math.cos(goal_direction - self.swing_joint))
        diff_deg = math.degrees(diff_rad)
        aligned = abs(diff_deg) <= self.angle_tolerance_deg

        # publish
        self.pub.publish(Bool(data=aligned))
        self.goal_direction_pub.publish(Float64(data=goal_direction))
        
        self.get_logger().info(
            f"swing_joint={math.degrees(self.swing_joint):.1f}°, goal_direction={math.degrees(goal_direction):.1f}°, diff={diff_deg:.1f}° → aligned={aligned}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SwingAlignmentChecker()
    rclpy.spin(node)
    rclpy.shutdown()
