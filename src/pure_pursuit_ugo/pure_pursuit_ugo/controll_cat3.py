import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

class ControllCat3(Node):
    def __init__(self):
        super().__init__('ControllCat3')
        
        self.declare_parameter('machine_name', 'cat303cr')
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.publish_period = 0.05   # Publish周期 (秒) ... 20Hz
        self.accumulate_period = 0.25   # 蓄積周期 (秒) ... 5Hz
        
        # Publish用タイマー (速い)
        self.publish_timer = self.create_timer(self.publish_period, self.publish_loop)
        # 蓄積用タイマー (遅い)
        self.accumulate_timer = self.create_timer(self.accumulate_period, self.accumulate_loop)
        # ---------------------------------

        self.joint_pub = self.create_publisher(JointState, f'{self.machine_name}/joint_command', 10)

        self.joint_names = [
            'swing_joint', 'boom_swing_joint', 'boom_joint', 'arm_joint', 
            'grapple_joint', 'upperfolk_joint', 'underfolk_joint', 'blade_joint'
        ]
        
        # 計算が行われる頻度で変化する量
        self.joint_speeds_rad = {
            'swing_joint': 0.05,
            'boom_swing_joint': 0.05,
            'boom_joint': 0.05,
            'arm_joint': 0.05,
            'grapple_joint': 0.05,
            'upperfolk_joint': 0.05,
            'underfolk_joint': 0.0,
            'blade_joint': 0.3
        }

        self.joint_positions = [0.0] * len(self.joint_names)
        self.last_joy = Joy()
        self.get_logger().info("ControllCat3Node (Separated Timers) Started")

    def joy_callback(self, msg):
        # joyの最新状態を保存するだけ
        self.last_joy = msg

    # --- 変更点: 蓄積(計算)専用のループ (遅い周期で実行) ---
    def accumulate_loop(self):
        if not self.last_joy.axes: # joyがまだ来ていない場合は何もしない
            return

        msg = self.last_joy
        # dtは「蓄積周期」を使う
        dt = self.accumulate_period 

        # --- 1. デジタルコマンド( -1.0, 0.0, 1.0 )の生成 ---
        
        # ★変更点: デッドゾーンを 0.0 に固定
        def get_digital_command(axis_index):
            val = msg.axes[axis_index]
            if val > 0.0:
                return 1.0
            elif val < 0.0:
                return -1.0
            else:
                return 0.0

        def get_button_command(open_btn_idx, close_btn_idx):
            return float(msg.buttons[open_btn_idx] - msg.buttons[close_btn_idx])

        commands = {
            'swing_joint':     get_digital_command(6), # axes[6]
            'boom_swing_joint':get_digital_command(0), # axes[0]
            'boom_joint':      get_digital_command(7), # axes[7]
            'arm_joint':       get_digital_command(1), # axes[1]
            'grapple_joint':   get_button_command(1, 2),
            'upperfolk_joint': get_button_command(3, 0),
            'blade_joint':     get_button_command(6, 7)
        }
        
        # --- 2. 目標角度の計算 (インチング) ---
        for i, name in enumerate(self.joint_names):
            if name == 'underfolk_joint':
                continue

            # コマンド(1/0/-1) * 変化量(rad)
            delta_angle = commands.get(name, 0.0) * self.joint_speeds_rad[name]
            self.joint_positions[i] += delta_angle

        # --- 3. 連動する関節の処理 ---
        try:
            upperfolk_idx = self.joint_names.index('upperfolk_joint')
            underfolk_idx = self.joint_names.index('underfolk_joint')
            self.joint_positions[underfolk_idx] = -self.joint_positions[upperfolk_idx]
        except ValueError:
            pass # 起動時にエラーが出ないように


    # --- 変更点: Publish専用のループ (速い周期で実行) ---
    def publish_loop(self):
        # ここでは計算は一切行わない
        
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        
        # 現在のself.joint_positionsをコピーして送信
        joint_state_msg.position = [pos for pos in self.joint_positions] 
        
        self.joint_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllCat3()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()