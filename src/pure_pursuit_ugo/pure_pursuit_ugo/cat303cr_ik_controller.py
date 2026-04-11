import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import math
import numpy as np

class Cat303crIkController(Node):
    def __init__(self):
        super().__init__('cat303cr_ik_controller')

        # --- パラメータ定義 ---
        self.declare_parameter('machine_name', 'cat303cr')
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        self.declare_parameter('base_gain', 0.5)

        # --- リンク長の定義 ---
        self.boom_pos = np.array([0, 1.33, 0.975])
        self.arm_pos = np.array([0, 2.995, 2.43])
        self.grapple_upper_pos = np.array([0, 1.791, 2.79])
        self.grapple_lower_pos = np.array([0, 0.831, 2.92])
        
        self.L1 = np.linalg.norm(self.arm_pos - self.boom_pos)
        self.L2 = np.linalg.norm(self.grapple_upper_pos - self.arm_pos)
        self.L3 = np.linalg.norm(self.grapple_lower_pos - self.grapple_upper_pos)

        self.base_gain = self.get_parameter('base_gain').get_parameter_value().double_value

        # --- 通信周り ---
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joint_pub = self.create_publisher(JointState, f'{self.machine_name}/joint_command', 10)
        self.zoom_pub = self.create_publisher(Float32, '/zoom_level', 10)

        self.timer = self.create_timer(0.05, self.publish_commands)

        # --- 関節名 ---
        # 0:swing, 1:boom_swing, 2:boom, 3:arm, 4:grapple, 5:rotate, 6:upper, 7:under, 8:blade
        self.joint_names = [
            'swing_joint', 'boom_swing_joint', 'boom_joint', 'arm_joint', 'grapple_upperbody_joint',
            'grapple_lowerbody_joint', 'upperfolk_joint', 'underfolk_joint', 'blade_joint'
        ]
        
        # リミット設定
        self.joint_limits_lower = [-math.pi/2, -0.88, -0.349, -1.48, -1.5, -math.pi, -0.9, -0.22, -0.29]
        self.joint_limits_upper = [ math.pi/2,  1.3,  1.57,  0.52,  2.0,  math.pi,  0.22, 0.9,  0.5]
        
        # 初期関節角度 (長さ9)
        self.joint_positions = [0.0] * len(self.joint_names)

        # --- 角度オフセット (Zero Pose定義) ---
        self.boom_ref_angle = np.radians(-50.0)   # 水平より上なのでプラス
        self.arm_ref_angle = np.radians(123.0)  # 曲げ角基準
        self.grapple_ref_angle = np.radians(8.0)

        self.last_joy = Joy()

        # --- IKの初期目標値を設定 ---
        self.target_goal_position = Point()
        self.target_goal_position.x = 2.0
        self.target_goal_position.y = 0.0 
        self.target_goal_position.z = 1.5
        
        # 真下は -90度 (-pi/2)
        self.target_end_effector_angle = math.pi / 2 

        # 初期値でIKを一回解く
        if self.try_solve_ik(self.target_goal_position.x, self.target_goal_position.y,
                            self.target_goal_position.z, self.target_end_effector_angle):
            self.get_logger().info("Initial IK Solved successfully.")
        else:
            self.get_logger().warn("Initial IK Failed. Using default joints.")

        self.get_logger().info(f"Controller Started. L1={self.L1:.2f}, L2={self.L2:.2f}, L3={self.L3:.2f}")

    def joy_callback(self, msg: Joy):
        self.last_joy = msg

    def publish_commands(self):
        # 安全対策: コントローラーの軸不足によるクラッシュ防止
        if not self.last_joy.axes or len(self.last_joy.axes) < 6:
            return
        if not self.last_joy.buttons or len(self.last_joy.buttons) < 8:
            return

        joy_msg = self.last_joy
        dt = self.timer.timer_period_ns / 1e9
        gain = self.base_gain

        # 1. 座標計算
        next_x = self.target_goal_position.x + (joy_msg.buttons[3] - joy_msg.buttons[0]) * gain * dt
        next_y = self.target_goal_position.y + (joy_msg.buttons[2] - joy_msg.buttons[1]) * gain * dt
        next_z = self.target_goal_position.z + (joy_msg.buttons[6] - joy_msg.buttons[7]) * gain * dt
        
        angle_gain = math.radians(60.0)
        next_angle = self.target_end_effector_angle
        # 右スティック(Axes[4])があれば使用
        if len(joy_msg.axes) > 4:
            next_angle += joy_msg.axes[4] * angle_gain * dt

        # 2. IK計算
        if self.try_solve_ik(next_x, next_y, next_z, next_angle):
            self.target_goal_position.x = next_x
            self.target_goal_position.y = next_y
            self.target_goal_position.z = next_z
            self.target_end_effector_angle = next_angle

        # 3. その他操作 (トリガー操作など)
        val_l2 = joy_msg.axes[2]
        val_r2 = joy_msg.axes[5]
        # L2/R2は通常 -1.0(押下) ~ 1.0(解放) なので、押した分だけ動くように補正
        self.joint_positions[5] += (-(val_l2 - 1) + (val_r2 - 1)) * gain * dt 
        
        self.joint_positions[6] += (joy_msg.buttons[4] - joy_msg.buttons[5]) * gain * dt        
        self.joint_positions[7] = -self.joint_positions[6]                                      

        # パブリッシュ
        joint_state_msg = JointState(name=self.joint_names, position=self.joint_positions)
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_state_msg)

    def try_solve_ik(self, x, y, z, alpha):
        try:
            # 現在の関節角度をコピー（サイズ9を維持）
            final_joints = list(self.joint_positions)

            # --- [Step 1] 旋回 (Swing) ---
            theta_swing = math.atan2(y, x)
            
            # --- [Step 2] 平面IK (幾何学計算) ---
            offset_x = math.sqrt(x**2 + y**2)
            A = offset_x - self.L3 * math.cos(alpha)
            B = z - self.L3 * math.sin(alpha)

            r_sq = A**2 + B**2
            max_reach_sq = (self.L1 + self.L2)**2
            min_reach_sq = (self.L1 - self.L2)**2
            margin = 0.01
            
            # 到達チェック
            if r_sq > (max_reach_sq - margin) or r_sq < (min_reach_sq + margin):
                # 届かない場合はログを出さずにFalse（操作感を損なわないため）
                return False

            # --- [Step 3] アーム・ブーム計算 ---
            # 余弦定理で曲げ角(cos)を計算
            cos_theta_arm_bend = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            cos_theta_arm_bend = max(-1.0, min(1.0, cos_theta_arm_bend))
            
            # 幾何学的な曲げ角 (0: Straight, pi: Folded)
            theta_arm_bend_geom = math.acos(cos_theta_arm_bend)

            # ブーム計算
            k1 = self.L1 + self.L2 * math.cos(theta_arm_bend_geom)
            k2 = self.L2 * math.sin(theta_arm_bend_geom)
            theta_boom_geom = math.atan2(B, A) - math.atan2(k2, k1)

            # バケット(グラップル)の曲げ角
            theta_grapple_bend_geom = alpha - theta_boom_geom - theta_arm_bend_geom


            # --- [Step 4] ロボットへの指令値変換 (時計回り正) ---
            
            # 0: Swing
            final_joints[0] = theta_swing

            # 2: Boom (Reference - Geometric)
            # 下がる(時計回り)とプラス
            final_joints[2] = theta_boom_geom - self.boom_ref_angle

            # 3: Arm (Geometric - Reference)  <-- ★ここを修正しました★
            # アームは「曲げ角」がGeomなので、伸びる(反時計)とマイナス、曲がる(時計)とプラス
            # Geom(52) - Ref(123) = -71 (マイナス＝伸びる方向) となり整合します
            final_joints[3] = theta_arm_bend_geom - self.arm_ref_angle

            # 4: Grapple (Reference - Geometric)
            final_joints[4] = theta_grapple_bend_geom  -self.grapple_ref_angle


            # --- [Step 5] リミットチェック (デバッグ出力付き) ---
            check_indices = [0, 2, 3, 4]
            joint_labels = {0: "Swing", 2: "Boom", 3: "Arm", 4: "Grapple"}

            for i in check_indices:
                if not (self.joint_limits_lower[i] <= final_joints[i] <= self.joint_limits_upper[i]):
                    # どの関節がエラーか詳細を表示
                    self.get_logger().warn(
                        f"Limit Fail [{joint_labels[i]}]: Val={math.degrees(final_joints[i]):.1f} deg "
                        f"Range=[{math.degrees(self.joint_limits_lower[i]):.1f}, {math.degrees(self.joint_limits_upper[i]):.1f}]"
                    )
                    return False 

            # チェック通過後に更新
            self.joint_positions = final_joints
            return True

        except Exception as e:
            self.get_logger().error(f"IK Error: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = Cat303crIkController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()