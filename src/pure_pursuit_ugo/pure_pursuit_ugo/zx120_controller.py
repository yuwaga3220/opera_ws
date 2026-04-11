import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class Zx120Controller(Node):
    def __init__(self):
        super().__init__('Zx120Controller')

        # --- パラメータ定義 ---
        # 建機操作用
        self.declare_parameter('machine_name', 'zx120')
        self.declare_parameter('base_gain', 0.5) # ズーム1倍の時の基準ゲイン

        # ズーム操作用
        self.declare_parameter('min_zoom', 1.0)
        self.declare_parameter('max_zoom', 5.0)
        self.declare_parameter('zoom_speed', 1.0)

        # パラメータを取得
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        self.base_gain = self.get_parameter('base_gain').get_parameter_value().double_value
        self.min_zoom = self.get_parameter('min_zoom').get_parameter_value().double_value
        self.max_zoom = self.get_parameter('max_zoom').get_parameter_value().double_value
        self.zoom_speed = self.get_parameter('zoom_speed').get_parameter_value().double_value

        # --- Subscriber ---
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # --- Publisher ---
        self.twist_pub = self.create_publisher(Twist, f'{self.machine_name}/tracks/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, f'{self.machine_name}/joint_command', 10)
        self.zoom_pub = self.create_publisher(Float64, '/zoom_level', 10)

        # --- タイマー ---
        timer_period = 0.05  # 50ms (20Hz)
        self.timer = self.create_timer(timer_period, self.publish_commands)

        # --- 内部変数 ---
        # 建機用
        self.joint_names = [
            'swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint'
        ]
        self.joint_positions = [0.0] * len(self.joint_names)

        # 初期値設定
        self.joint_positions[0] = 0.0
        self.joint_positions[1] = - math.pi / 6
        self.joint_positions[2] = math.pi / 3
        self.joint_positions[3] = math.pi / 6

        # 関節の可動域（上限・下限）を定義（要調整）
        self.joint_limits_lower = [
            -math.pi / 2,   # swing_joint (-90 degrees)
            -math.pi / 2,            # boom_joint
            0.0,            # arm_joint (-90 degrees)
            0.0,    # bucket_joint
        ]
        self.joint_limits_upper = [
            math.pi / 2,    # swing_joint (+90 degrees)
            0.0,           # boom_joint
            math.pi * 3 / 4,           # arm_joint
            math.pi / 2,    # bucket_joint

        ]

        # ズーム用
        self.current_zoom_level = 1.0
        self.zoom_direction = 0.0 # D-padの入力

        # 共有
        self.last_joy = Joy()

        self.get_logger().info("Zx120Controller Started")
        self.get_logger().info(f"Machine: {self.machine_name}, Control Gain (base): {self.base_gain}")
        self.get_logger().info(f"Zoom control: D-pad Up/Down, Range: [{self.min_zoom}, {self.max_zoom}]")
        self.get_logger().info("Joint limits have been configured.") # ログを追加

    def joy_callback(self, msg):
        """Joyスティックのデータを受信して保存"""
        self.last_joy = msg

        zoom_in_pressed = False
        zoom_out_pressed = False

        # ボタンの数が足りているかチェック
        if len(msg.buttons) > 10:
            zoom_in_pressed = (msg.buttons[10] == 1) # ズームインボタン
            zoom_out_pressed = (msg.buttons[9] == 1) # ズームアウトボタン

        # 押されたボタンに応じてズーム方向を決定
        if zoom_in_pressed:
            self.zoom_direction = 1.0
        elif zoom_out_pressed:
            self.zoom_direction = -1.0
        else:
            self.zoom_direction = 0.0

    def publish_commands(self):
        """一定周期で各種計算を行い、全トピックをPublishする"""
        if not self.last_joy.axes:  # joyがまだ来ていない場合は何もしない
            return
        
        joy_msg = self.last_joy
        
        # 建機の移動制御（Twist）
        twist = Twist()
        twist.linear.x = joy_msg.axes[7] * 1.5
        twist.angular.z = joy_msg.axes[6] * 1.5
        self.twist_pub.publish(twist)

        # ▼▼▼ 1. ズーム倍率の計算してpub
        dt = self.timer.timer_period_ns / 1e9  # 秒単位
        zoom_change = self.zoom_direction * self.zoom_speed * dt
        self.current_zoom_level += zoom_change
        self.current_zoom_level = max(self.min_zoom, min(self.max_zoom, self.current_zoom_level))

        zoom_msg = Float64()
        zoom_msg.data = self.current_zoom_level
        self.zoom_pub.publish(zoom_msg)

        # ▼▼▼ 2. 操作ゲインの計算（反比例） ▼▼▼
        # gain = self.base_gain / self.current_zoom_level
        gain = self.base_gain

        # ▼▼▼ 3. 建機の関節角度の計算して宣言してpub
        self.joint_positions[0] += joy_msg.axes[0] * gain * dt      # swing
        self.joint_positions[1] += joy_msg.axes[4] * gain * dt      # boom
        self.joint_positions[2] -= joy_msg.axes[1] * gain * dt      # arm
        self.joint_positions[3] += joy_msg.axes[3] * gain * dt      # bucket

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

        # ゲインの変化をデバッグ表示
        # self.get_logger().info(f"Zoom: {self.current_zoom_level:.2f}, Gain: {gain:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = Zx120Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()