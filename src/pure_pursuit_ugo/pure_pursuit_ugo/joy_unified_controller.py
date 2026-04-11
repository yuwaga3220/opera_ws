import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Point
import math as mt
import numpy as np

class JoyUnifiedControllerNode(Node):
    def __init__(self):
        super().__init__('joy_unified_controller_node')

        # ### パラメータとリンク長設定 ###
        self.declare_parameter('machine_name', 'zx120')
        machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        
        # 既知の座標からリンク長を計算
        self.boom_pos = np.array([12.14, 1.11, 0])
        self.arm_pos = np.array([12.14, 1.26, 4.8])
        self.bucket_pos = np.array([12.14, 1.26, 7.35])
        self.bucket_end_pos = np.array([12.14, 1.293, 8.48])
        self.L1 = np.linalg.norm(self.arm_pos - self.boom_pos)       # ブームの長さ
        self.L2 = np.linalg.norm(self.bucket_pos - self.arm_pos)     # アームの長さ
        self.L3 = np.linalg.norm(self.bucket_end_pos - self.bucket_pos) # バケットの長さ

        # ### Publisher ###
        self.swing_pub = self.create_publisher(Float64, f'{machine_name}/swing/cmd', 10)
        self.boom_pub = self.create_publisher(Float64, f'{machine_name}/boom/cmd', 10)
        self.arm_pub = self.create_publisher(Float64, f'{machine_name}/arm/cmd', 10)
        self.bucket_pub = self.create_publisher(Float64, f'{machine_name}/bucket/cmd', 10)
        # 必要に応じて車両の速度Publisherも追加
        # self.cmd_vel_pub = self.create_publisher(Twist, f'{machine_name}/cmd_vel', 10)

        # ### Subscriber ###
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joint_states_sub = self.create_subscription(JointState, f'/{machine_name}/joint_states', self.joint_states_callback, 10)

        # ### 状態変数 ###
        # 目標値
        self.target_goal_position = Point()
        self.target_swing_angle = 0.0
        self.target_boom_angle = 0.0
        self.target_arm_angle = 0.0
        self.target_bucket_angle = 0.0
        
        # 現在の観測値
        self.current_joint_angles = {} # swing, boom, arm, bucketの現在角度を保持
        self.last_joy_msg = Joy()
        self.is_initialized = False

        # ### タイマー ###
        self.timer = self.create_timer(0.05, self.publish_commands)  # 20Hz

        self.get_logger().info("Unified Controller Node has been started.")

    def joint_states_callback(self, msg: JointState):
        # 現在の関節角度を更新
        for i, name in enumerate(msg.name):
            self.current_joint_angles[name] = msg.position[i]

        # 最初の1回だけ、現在の関節角度を目標の初期値に設定
        if not self.is_initialized and 'swing_joint' in self.current_joint_angles:
            self.target_swing_angle = self.current_joint_angles.get('swing_joint', 0.0)
            self.target_boom_angle = self.current_joint_angles.get('boom_joint', mt.radians(-45)) # 初期姿勢に近い値
            self.target_arm_angle = self.current_joint_angles.get('arm_joint', mt.radians(90))
            self.target_bucket_angle = self.current_joint_angles.get('bucket_joint', mt.radians(0))

            # 初期状態から順運動学で先端位置を計算
            self.update_goal_position_from_fk()
            self.is_initialized = True
            self.get_logger().info(f"Initialized with joint states. Initial goal: {self.target_goal_position.x:.2f}, {self.target_goal_position.y:.2f}, {self.target_goal_position.z:.2f}")


    def joy_callback(self, msg):
        self.last_joy_msg = msg

    def publish_commands(self):
        if not self.is_initialized:
            self.get_logger().warn("Waiting for initial joint states...")
            return

        msg = self.last_joy_msg
        dt = 0.05  # timer周期
        
        # ゲイン設定
        position_gain = 0.5 
        angle_gain = mt.radians(20.0) # 1秒で20度動く速さ

        # --- モード判定と状態更新 ---
        # msg.axes[0]~[2]は左スティック, [3]~[5]は右スティック
        # 右スティックが操作されたら、タスク空間操作
        if abs(msg.axes[4]) > 0.1 or abs(msg.axes[3]) > 0.1 or msg.buttons[5] != 0 or msg.buttons[4] != 0:
            self.update_goal_from_joy(msg, position_gain, dt)
            self.update_angles_from_ik()
        
        # 十字キーなどが操作されたら、ジョイント空間操作
        elif abs(msg.axes[1]) > 0.1 or abs(msg.axes[0]) > 0.1 or msg.buttons[6] != 0 or msg.buttons[7] != 0:
            self.update_angles_from_joy(msg, angle_gain, dt)
            self.update_goal_position_from_fk()
        
        # --- パブリッシュ ---
        self.swing_pub.publish(Float64(data=self.target_swing_angle))
        self.boom_pub.publish(Float64(data=self.target_boom_angle))
        self.arm_pub.publish(Float64(data=self.target_arm_angle))
        self.bucket_pub.publish(Float64(data=self.target_bucket_angle))
        
        # ログ出力
        self.get_logger().info(
            f"Target Goal: x={self.target_goal_position.x:.2f}, y={self.target_goal_position.y:.2f}, z={-self.target_goal_position.z:.2f} | "
            f"Target Angles: Sw={mt.degrees(self.target_swing_angle):.1f}, Bm={mt.degrees(self.target_boom_angle):.1f}, Am={mt.degrees(self.target_arm_angle):.1f}, Bk={mt.degrees(self.target_bucket_angle):.1f}"
        )

    def update_goal_from_joy(self, msg, gain, dt):
        """ジョイコン入力に基づき、目標先端位置を更新する"""
        self.target_goal_position.x += msg.axes[4] * gain * dt  # 右スティック上下
        self.target_goal_position.y += msg.axes[3] * gain * dt  # 右スティック左右
        self.target_goal_position.z -= msg.buttons[5] * gain * dt  # Rボタン
        self.target_goal_position.z += msg.buttons[4] * gain * dt  # Lボタン
        self.get_logger().info("Mode: Task Space Control (Tip Goal)")

    def update_angles_from_joy(self, msg, gain, dt):
        """ジョイコン入力に基づき、目標関節角度を更新する"""
        # 十字キー上下でブーム
        self.target_boom_angle += msg.axes[1] * gain * dt 
        # 十字キー左右でアーム
        self.target_arm_angle += msg.axes[0] * gain * dt
        # start, backボタンでバケット
        self.target_bucket_angle += msg.buttons[6] * gain * dt
        self.target_bucket_angle -= msg.buttons[7] * gain * dt
        
        # swingは別途指定が良いかも（例：ZL/ZRボタン）
        # self.target_swing_angle += ...

        self.get_logger().info("Mode: Joint Space Control")


    def update_angles_from_ik(self):
        """現在の目標位置から逆運動学(IK)を解き、目標関節角度を更新する"""
        try:
            x = self.target_goal_position.x
            y = self.target_goal_position.y
            z = self.target_goal_position.z
            
            # Swing角の計算
            self.target_swing_angle = mt.atan2(y, x)
            
            # 2D平面上でのIK計算
            # 旋回後の座標系に変換
            offset_x = mt.sqrt(x**2 + y**2)
            alpha = self.target_boom_angle + self.target_arm_angle + self.target_bucket_angle # 水平からのバケット先端の絶対角度
            
            A = offset_x - self.L3 * mt.cos(alpha) # mt.sinからcosへ変更(角度定義による)
            B = z - self.L3 * mt.sin(alpha)      # mt.cosからsinへ変更

            # 到達範囲チェック
            r_sq = A**2 + B**2
            if r_sq > (self.L1 + self.L2)**2 or r_sq < (self.L1 - self.L2)**2:
                self.get_logger().warn("IK: Target is out of reach for arm and boom.")
                return

            # cos(theta_arm)を計算
            cos_theta_arm = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            
            # アーム角度 (エルボーアップ解)
            self.target_arm_angle = mt.acos(cos_theta_arm)
            
            # ブーム角度
            k1 = self.L1 + self.L2 * mt.cos(self.target_arm_angle)
            k2 = self.L2 * mt.sin(self.target_arm_angle)
            self.target_boom_angle = mt.atan2(B, A) - mt.atan2(k2, k1)

            # バケット角度はalphaから逆算
            self.target_bucket_angle = alpha - self.target_boom_angle - self.target_arm_angle

        except Exception as e:
            self.get_logger().error(f"Error during IK calculation: {str(e)}")


    def update_goal_position_from_fk(self):
        """現在の目標関節角度から順運動学(FK)を解き、目標先端位置を更新する"""
        # ブーム、アーム、バケットの角度から2D平面上の先端位置を計算
        x_plane = (self.L1 * mt.cos(self.target_boom_angle) + 
                   self.L2 * mt.cos(self.target_boom_angle + self.target_arm_angle) + 
                   self.L3 * mt.cos(self.target_boom_angle + self.target_arm_angle + self.target_bucket_angle))
        
        z_plane = (self.L1 * mt.sin(self.target_boom_angle) + 
                   self.L2 * mt.sin(self.target_boom_angle + self.target_arm_angle) + 
                   self.L3 * mt.sin(self.target_boom_angle + self.target_arm_angle + self.target_bucket_angle))

        # Swing角を適用して3D座標に変換
        self.target_goal_position.x = x_plane * mt.cos(self.target_swing_angle)
        self.target_goal_position.y = x_plane * mt.sin(self.target_swing_angle)
        self.target_goal_position.z = z_plane

def main(args=None):
    rclpy.init(args=args)
    node = JoyUnifiedControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()