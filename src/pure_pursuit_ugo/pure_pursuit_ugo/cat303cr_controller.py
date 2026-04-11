import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64
import math

class Cat303crController(Node):
    def __init__(self):
        super().__init__('cat303cr_controller')

        # --- パラメータ定義 ---
        self.declare_parameter('machine_name', 'cat303cr')
        self.declare_parameter('base_gain', 0.5)

        # ズームの想定レンジ（安全用クランプ）
        self.declare_parameter('min_zoom', 1.0)
        self.declare_parameter('max_zoom', 5.0)

        # 0除算防止（念のため）
        self.declare_parameter('min_zoom_for_div', 0.05)

        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        self.base_gain = self.get_parameter('base_gain').get_parameter_value().double_value
        self.min_zoom = self.get_parameter('min_zoom').get_parameter_value().double_value
        self.max_zoom = self.get_parameter('max_zoom').get_parameter_value().double_value
        self.min_zoom_for_div = self.get_parameter('min_zoom_for_div').get_parameter_value().double_value

        # --- Subscriber ---
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # ★ Unityなどからのズーム倍率（最後に受信した値を保持）
        self.zoom_sub = self.create_subscription(Float64, '/cat_zoom_level', self.zoom_callback, 10)

        # --- Publisher ---
        self.joint_pub = self.create_publisher(JointState, f'{self.machine_name}/joint_command', 10)

        # --- タイマー ---
        self.timer = self.create_timer(0.2, self.publish_commands)  # 5Hz

        # --- 関節配列 ---
        self.joint_names = [
            'swing_joint', 'boom_swing_joint', 'boom_joint', 'arm_joint', 'grapple_upperbody_joint',
            'grapple_lowerbody_joint', 'upperfolk_joint', 'underfolk_joint', 'blade_joint'
        ]
        # 関節位置の初期値
        self.joint_positions = [0.0] * len(self.joint_names)

        # 初期値
        self.joint_positions[2] = 0.275
        self.joint_positions[3] = -0.05
        self.joint_positions[8] = 0.2

        self.joint_limits_lower = [
            -math.pi / 2, -0.88, -0.34, -1.48, -1.5, -math.pi, -0.9, -0.22, -0.29
        ]
        self.joint_limits_upper = [
            math.pi / 2,  1.3,   1.57,  0.52,  2.0,   math.pi, 0.22, 0.9,  0.5
        ]

        # 最後に受信したズーム倍率（受信前は 1.0 のまま = “最後の値”）
        self.current_zoom_level = 1.0

        # 共有
        self.last_joy = Joy()

        self.get_logger().info("Cat303crController Started")
        self.get_logger().info("Using last received zoom level from /cat_zoom_level for gain computation.")

    def joy_callback(self, msg: Joy):
        self.last_joy = msg

    def zoom_callback(self, msg: Float64):
        # 受信したズームを保持（= 最後に受信した値として使い続ける）
        z = float(msg.data)
        z = max(self.min_zoom, min(self.max_zoom, z))   # 安全クランプ
        self.current_zoom_level = z
        self.get_logger().info(f"Received zoom level: {self.current_zoom_level}")

    def publish_commands(self):
        if not self.last_joy.axes:
            return

        dt = self.timer.timer_period_ns / 1e9

        # ★ 最後に受信したズーム倍率でゲイン計算（反比例）
        zoom = max(self.current_zoom_level, self.min_zoom_for_div)  # 0除算防止
        zoom = (zoom - 1.0) * 2.0 + 1.0  # gain計算用のズーム倍率を調整
        gain = self.base_gain # / zoom

        joy_msg = self.last_joy
        self.joint_positions[0] += joy_msg.axes[0] * gain * dt
        self.joint_positions[2] += joy_msg.axes[4] * gain * dt
        self.joint_positions[3] -= joy_msg.axes[1] * gain * dt
        self.joint_positions[4] += joy_msg.axes[3] * gain * dt
        self.joint_positions[5] += (-(joy_msg.axes[2] - 1) + (joy_msg.axes[5] - 1)) * gain * dt
        self.joint_positions[6] += (joy_msg.buttons[4] - joy_msg.buttons[5]) * gain * dt
        self.joint_positions[7]  = -self.joint_positions[6]

        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = max(
                self.joint_limits_lower[i],
                min(self.joint_limits_upper[i], self.joint_positions[i])
            )

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        self.joint_pub.publish(joint_state_msg)
        self.get_logger().info(f"gain : {gain}")

def main(args=None):
    rclpy.init(args=args)
    node = Cat303crController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
