import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import math
import numpy as np

class UnifiedControllerWithZoom(Node):
    def __init__(self):
        super().__init__('unified_controller_with_zoom')

        # --- パラメータ定義 ---
        self.declare_parameter('machine_name', 'cat303cr')
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value

        # 操作ゲイン・ズーム用
        self.declare_parameter('base_gain', 0.5)
        self.declare_parameter('min_zoom', 1.0)
        self.declare_parameter('max_zoom', 5.0)
        self.declare_parameter('zoom_speed', 1.0)

        # 既知の座標からリンク長を計算
        self.boom_pos = np.array([0, 1.33, 0.975])
        self.arm_pos = np.array([0, 2.995, 2.43])
        self.grapple_upper_pos = np.array([0., 1.967, 2.753])
        self.grapple_lower_pos = np.array([0, 0.612, 2.92])
        
        # IK用リンク長 (メートル単位) - 建機に合わせて正確な値に設定してください
        self.declare_parameter('link_boom', np.linalg.norm(self.arm_pos - self.boom_pos))    # L1: ブームの長さ
        self.declare_parameter('link_arm', np.linalg.norm(self.grapple_upper_pos - self.arm_pos))     # L2: アームの長さ
        self.declare_parameter('link_grapple', np.linalg.norm(self.grapple_lower_pos - self.grapple_upper_pos)) # L3: グラップルの長さ

        # パラメータ取得
        self.base_gain = self.get_parameter('base_gain').get_parameter_value().double_value
        self.min_zoom = self.get_parameter('min_zoom').get_parameter_value().double_value
        self.max_zoom = self.get_parameter('max_zoom').get_parameter_value().double_value
        self.zoom_speed = self.get_parameter('zoom_speed').get_parameter_value().double_value
        self.L1 = self.get_parameter('link_boom').get_parameter_value().double_value
        self.L2 = self.get_parameter('link_arm').get_parameter_value().double_value
        self.L3 = self.get_parameter('link_grapple').get_parameter_value().double_value

        # --- Subscriber ---
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # --- Publisher ---
        self.joint_pub = self.create_publisher(JointState, f'{self.machine_name}/joint_command', 10)
        self.zoom_pub = self.create_publisher(Float32, '/zoom_level', 10)

        # --- タイマー ---
        timer_period = 0.05  # 50ms (20Hz)
        self.timer = self.create_timer(timer_period, self.publish_commands)

        # --- 内部変数 ---
        self.joint_names = [
            'swing_joint', 'boom_swing_joint', 'boom_joint', 'arm_joint', 'grapple_upperbody_joint',
            'grapple_lowerbody_joint', 'upperfolk_joint', 'underfolk_joint', 'blade_joint'
        ]
        
        # 初期値設定
        self.joint_positions = [0.0] * len(self.joint_names)
        # self.joint_positions[1] = np.radians(10)
        # self.joint_positions[2] = -np.radians(0)
        # self.joint_positions[8] = -np.radians(20)
        
        # 計算するときの角度と実際の角度の差
        self.boom_gap = np.radians(60)
        self.arm_gap = np.radians(100)
        self.grapple_gap = np.radians(0)
        
        # 限界角度設定
        self.joint_limits_lower = [-math.pi / 2, -0.88, -0.34, -1.48, -0.68, -3.14, -0.9, -0.22, -0.29]
        self.joint_limits_upper = [math.pi / 2, 1.3, 1.57, 0.52, 2.0, 3.14, 0.22, 0.9, 0.5]

        # ズーム用
        self.current_zoom_level = 1.0
        self.zoom_direction = 0.0

        # IK (タスク空間操作) 用
        self.control_mode = 'joint'
        self.target_goal_position = Point()
        self.target_end_effector_angle = np.radians(180)  # 先端の絶対角度(alpha)
        self.is_ik_initialized = False
        self.last_mode_switch_button_state = 0
        self.last_joy = Joy()

        self.get_logger().info("Unified Controller with 3-Link IK Started")
        self.get_logger().info(f"Links: L1={self.L1}, L2={self.L2}, L3={self.L3}")
        self.get_logger().info("Press 'START/MENU' button to switch control mode.")
        
        # ★★★ 新規：定義済みの指令角度をもとにIKを即座に初期化 ★★★
        self.get_logger().info("定義済みの指令角度でIKを初期化します...")
        self.update_goal_from_fk()  # joint_positionsの初期値から目標座標を計算
        self.get_logger().info(f"IK Initialized. Tip Pos: [x={self.target_goal_position.x:.2f}, y={self.target_goal_position.y:.2f}, z={self.target_goal_position.z:.2f}], Tip Angle: {math.degrees(self.target_end_effector_angle):.1f} deg")
        self.get_logger().info("Unified Controller with 3-Link IK Started")
        self.get_logger().info(f"Links: L1={self.L1}, L2={self.L2}, L3={self.L3}")
        self.get_logger().info("Press 'START/MENU' button to switch control mode.")

    def joy_callback(self, msg: Joy):
        self.last_joy = msg
        
        # --- ズーム操作 ---
        zoom_in_pressed = (len(msg.buttons) > 10 and msg.buttons[10] == 1)
        zoom_out_pressed = (len(msg.buttons) > 9 and msg.buttons[9] == 1)
        if zoom_in_pressed: self.zoom_direction = 1.0
        elif zoom_out_pressed: self.zoom_direction = -1.0
        else: self.zoom_direction = 0.0
        
        # --- モード切替操作 ---
        current_button_state = msg.buttons[8] if len(msg.buttons) > 8 else 0 # Logicoolボタン
        if current_button_state == 1 and self.last_mode_switch_button_state == 0:
            if self.control_mode == 'joint':
                self.control_mode = 'task'
                # モード切替時に現在の姿勢から目標値を再設定
                self.update_goal_from_fk()
                self.get_logger().info("Switched to TASK SPACE (3-Link IK) control.")
            else:
                self.control_mode = 'joint'
                self.get_logger().info("Switched to JOINT SPACE control.")
        self.last_mode_switch_button_state = current_button_state

    def publish_commands(self):
        if not self.last_joy.axes: return

        joy_msg = self.last_joy
        dt = self.timer.timer_period_ns / 1e9
        gain = self.base_gain / self.current_zoom_level # ズーム連動ゲイン

        if self.control_mode == 'joint':
            # --- 関節空間での操作 ---
            self.joint_positions[0] += joy_msg.axes[0] * gain * dt                                  # swing
            self.joint_positions[2] += joy_msg.axes[1] * gain * dt                                  # boom
            self.joint_positions[3] -= joy_msg.axes[4] * gain * dt                                  # arm
            self.joint_positions[4] += joy_msg.axes[3] * gain * dt                                  # grapple_upperbody_joint
            self.joint_positions[5] += (-(joy_msg.axes[2] - 1) + (joy_msg.axes[5] - 1)) * gain * dt # grapple_lowerbody_joint
            self.joint_positions[6] += (joy_msg.buttons[4] - joy_msg.buttons[5]) * gain * dt        # upperfolk
            self.joint_positions[7] = -self.joint_positions[6]

        elif self.control_mode == 'task':
            # --- タスク空間での操作 ---
            # 先端位置・角度の更新 (右スティック, B/X)
            self.target_goal_position.x += joy_msg.axes[4] * gain * dt
            self.target_goal_position.y += joy_msg.axes[3] * gain * dt
            self.target_goal_position.z += (joy_msg.buttons[5] - joy_msg.buttons[4]) * gain * dt
            angle_gain = math.radians(60.0) # 1秒で45度
            self.target_end_effector_angle += (joy_msg.buttons[1] - joy_msg.buttons[2]) * angle_gain * dt

            # IKを解いて swing, boom, arm, grapple の角度を更新
            self.update_angles_from_ik()

        # # 関節可動域制限
        # for i in range(len(self.joint_positions)):
        #     self.joint_positions[i] = max(self.joint_limits_lower[i], min(self.joint_limits_upper[i], self.joint_positions[i]))

        # パブリッシュ
        joint_state_msg = JointState(name=self.joint_names, position=self.joint_positions)
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_state_msg)
        
        # # ログ出力
        # self.get_logger().info(
        #     f"Target Goal: x={self.target_goal_position.x:.2f}, y={self.target_goal_position.y:.2f}, z={self.target_goal_position.z:.2f} | "
        #     f"Target Angles: Sw={math.degrees(self.target_end_effector_angle):.1f}"
        # )

    def update_angles_from_ik(self):
        """目標位置（と目標角度）から3リンクIKを解き、関節角度を更新"""
        try:
            x, y, z = self.target_goal_position.x, self.target_goal_position.y, self.target_goal_position.z
            alpha = self.target_end_effector_angle
            
            # 1. 旋回角(swing)の計算
            theta_swing = math.atan2(y, x)
            
            # 2. 旋回後の2D平面上での手首(wrist)の目標座標(A, B)を計算
            x_plane = math.sqrt(x**2 + y**2) # 旋回中心からの水平距離
            A = x_plane - self.L3 * math.cos(alpha)
            B = z - self.L3 * math.sin(alpha)

            # 3. 2リンクIKでブームとアームの角度を計算 要修正
            r_sq = A**2 + B**2
            # if not ((self.L1 - self.L2)**2 <= r_sq <= (self.L1 + self.L2)**2):
            #     self.get_logger().warn("IK: Wrist target is out of reach for boom and arm.", throttle_duration_sec=1)
            #     return

            cos_theta_arm = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            cos_theta_arm = max(-1.0, min(1.0, cos_theta_arm))
            theta_arm = math.acos(cos_theta_arm) # エルボーアップ解

            k1 = self.L1 + self.L2 * math.cos(theta_arm)
            k2 = self.L2 * math.sin(theta_arm)
            theta_boom = math.atan2(B, A) - math.atan2(k2, k1)

            # 4. グラップル関節の相対角度を計算
            theta_grapple = alpha - theta_boom - theta_arm

            # 5. 計算結果を反映（実際の値に直す）
            self.joint_positions[0] = theta_swing
            self.joint_positions[2] = theta_boom - self.boom_gap
            self.joint_positions[3] = theta_arm - self.arm_gap
            self.joint_positions[4] = theta_grapple - self.grapple_gap
            
            # self.get_logger().info(
            #     f"point: {x:.2f}, {y:.2f}, {z:.2f} | "
            # )
            # self.get_logger().info(
            #     f"joint: {np.degrees(self.joint_positions[2]):.2f}, {np.degrees(self.joint_positions[3]):.2f}, {np.degrees(self.joint_positions[4]):.2f} | "
            # )   

            
        except (ValueError, ZeroDivisionError) as e:
            self.get_logger().error(f"Error during IK calculation: {e}", throttle_duration_sec=1)

    def update_goal_from_fk(self):
        """現在の目標関節角度からFKを解き、目標位置を更新"""
        # 実際の値を計算用に直す
        theta_boom = self.joint_positions[2] + self.boom_gap
        theta_arm = self.joint_positions[3] + self.arm_gap
        theta_grapple = self.joint_positions[4] + self.grapple_gap

        # 先端の絶対角度
        self.target_end_effector_angle = theta_boom + theta_arm + theta_grapple
        
        # self.get_logger().info(
        #     f"{np.degrees(self.joint_positions[2]):.2f}, {np.degrees(self.joint_positions[3]):.2f}, {np.degrees(self.joint_positions[4]):.2f} | "
        #     f"{np.degrees(theta_boom):.2f}, {np.degrees(theta_arm):.2f}, {np.degrees(theta_grapple):.2f} | "
        #     f"{np.degrees(self.target_end_effector_angle):.2f}"
        # )
        
        # 2D平面上の先端位置
        x_plane = (self.L1 * math.sin(theta_boom) + 
                   self.L2 * math.sin(theta_boom + theta_arm) + 
                   self.L3 * math.sin(self.target_end_effector_angle))
        z_plane = (self.L1 * math.cos(theta_boom) + 
                   self.L2 * math.cos(theta_boom + theta_arm) + 
                   self.L3 * math.cos(self.target_end_effector_angle))

        # 3D座標に変換
        self.target_goal_position.x = x_plane / math.cos(self.joint_positions[0])
        self.target_goal_position.y = x_plane * math.sin(self.joint_positions[0])
        self.target_goal_position.z = z_plane

        self.get_logger().info(
            f"joint: {np.degrees(self.joint_positions[2]):.2f}, {np.degrees(self.joint_positions[3]):.2f}, {np.degrees(self.joint_positions[4]):.2f} | "
        )   
        self.get_logger().info(
            f"point: {self.target_goal_position.x:.2f}, {self.target_goal_position.y:.2f}, {self.target_goal_position.z:.2f} | "
        )
        
def main(args=None):
    rclpy.init(args=args)
    node = UnifiedControllerWithZoom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()