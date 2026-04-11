import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math
import numpy as np

class Zx120IkController(Node):
    def __init__(self):
        super().__init__('zx120_ik_controller')

        # --- パラメータ定義 ---
        self.declare_parameter('machine_name', 'zx120')
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        self.declare_parameter('base_gain', 0.5)

        # --- リンク長の定義 ---
        # 座標から長さを計算（変更なし）
        self.boom_pos = np.array([12.14, 1.11, 0])
        self.arm_pos = np.array([12.14, 1.26, 4.8])
        self.bucket_pos = np.array([12.14, 1.26, 7.35])
        self.bucket_end_pos = np.array([12.14, 1.293, 8.48])
        
        self.L1 = np.linalg.norm(self.arm_pos - self.boom_pos)
        self.L2 = np.linalg.norm(self.bucket_pos - self.arm_pos)
        self.L3 = np.linalg.norm(self.bucket_end_pos - self.bucket_pos)

        self.base_gain = self.get_parameter('base_gain').get_parameter_value().double_value

        # --- Subscriber / Publisher ---
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joint_pub = self.create_publisher(JointState, f'{self.machine_name}/joint_command', 10)

        # --- タイマー ---
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.publish_commands)

        # --- 内部変数 ---
        self.joint_names = ['swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint']
        
        # リミット設定
        self.joint_limits_lower = [-math.pi, -math.pi, 0.0, -math.pi / 2]
        self.joint_limits_upper = [ math.pi,  0.0,  math.pi,  math.pi]
        
        # 初期関節角度（すべて0だとブームが水平になり、制限ギリギリなので安全な初期値を入れる）
        # ※ ブームを少し下げた状態などを初期値にするのが安全です
        self.joint_positions = [0.0, -0.5, 1.5, 0.0] 

        self.last_joy = Joy()

        # --- IKの初期目標値を設定 ---
        self.target_goal_position = Point()
        self.target_goal_position.x = 5.0  # 少し遠くに
        self.target_goal_position.y = 0.0 
        self.target_goal_position.z = 0.0 # 地面付近に
        self.target_end_effector_angle = math.pi / 2

        # 初期値でIKを一回解く（失敗してもjoint_positionsは上記の手動初期値が残るようにする）
        if self.try_solve_ik(self.target_goal_position.x, self.target_goal_position.y,
                            self.target_goal_position.z, self.target_end_effector_angle):
            self.get_logger().info("Initial IK Solved successfully.")
        else:
            self.get_logger().warn("Initial IK Failed (Target out of reach or limit). using default joints.")

        self.get_logger().info(f"Controller Started. L1={self.L1:.2f}, L2={self.L2:.2f}, L3={self.L3:.2f}")

    def joy_callback(self, msg: Joy):
        self.last_joy = msg

    def publish_commands(self):
        # Joyがまだ来ていない、または空の場合は何もしない
        if not self.last_joy.axes: return

        joy_msg = self.last_joy
        dt = self.timer.timer_period_ns / 1e9
        gain = self.base_gain

        # --- 1. 次の候補座標を計算 ---
        next_x = self.target_goal_position.x + (joy_msg.buttons[3] - joy_msg.buttons[0]) * gain * dt
        next_y = self.target_goal_position.y + (joy_msg.buttons[2] - joy_msg.buttons[1]) * gain * dt
        next_z = self.target_goal_position.z + (joy_msg.buttons[6] - joy_msg.buttons[7]) * gain * dt
        
        angle_gain = math.radians(60.0)
        next_angle = self.target_end_effector_angle + joy_msg.axes[4] * angle_gain * dt

        # --- 2. IKを試算 ---
        if self.try_solve_ik(next_x, next_y, next_z, next_angle):
            # 成功時のみターゲット座標を更新
            self.target_goal_position.x = next_x
            self.target_goal_position.y = next_y
            self.target_goal_position.z = next_z
            self.target_end_effector_angle = next_angle
        else:
            # 失敗時はターゲットを更新しない（＝壁に当たった挙動）
            # デバッグ用にログを出すとわかりやすい（頻繁に出るのでthrottleを入れる）
            pass

        # --- 3. パブリッシュ（重要：ifの外に出す） ---
        # IKが成功してもしなくても、"現在の有効な角度" を送り続ける必要があります
        joint_state_msg = JointState(name=self.joint_names, position=self.joint_positions)
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_state_msg)

    def try_solve_ik(self, x, y, z, alpha):
        try:
            temp_joints = [0.0] * 4

            # Swing
            temp_joints[0] = math.atan2(y, x)
            
            # 2D Plane IK
            offset_x = math.sqrt(x**2 + y**2)
            A = offset_x - self.L3 * math.cos(alpha)
            B = z - self.L3 * math.sin(alpha)

            r_sq = A**2 + B**2
            max_reach_sq = (self.L1 + self.L2)**2
            min_reach_sq = (self.L1 - self.L2)**2 # ここは単純差分だと不正確な場合があるが一旦OK
            
            # 到達範囲チェック（少しマージンを持つ）
            margin = 0.01
            if r_sq > (max_reach_sq - margin) or r_sq < (min_reach_sq + margin):
                return False

            cos_theta_arm = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            cos_theta_arm = max(-1.0, min(1.0, cos_theta_arm))

            # Arm angle（エルボーアップであり、acosなので正しかでてこない）
            # 出てくるのは曲げ角で角度定義はここで合う
            temp_joints[2] = math.acos(cos_theta_arm)

            k1 = self.L1 + self.L2 * math.cos(temp_joints[2])
            k2 = self.L2 * math.sin(temp_joints[2])
            temp_joints[1] = math.atan2(B, A) - math.atan2(k2, k1) # Boom angle

            temp_joints[3] = alpha - temp_joints[1] - temp_joints[2] # Bucket angle

            # リミットチェック
            for i in range(4):
                if not (self.joint_limits_lower[i] <= temp_joints[i] <= self.joint_limits_upper[i]):
                    return False 

            self.joint_positions = temp_joints
            return True

        except Exception as e:
            self.get_logger().error(f"IK Error: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = Zx120IkController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()