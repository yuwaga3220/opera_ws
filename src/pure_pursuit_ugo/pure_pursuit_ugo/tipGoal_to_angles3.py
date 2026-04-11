import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import math as mt
import numpy as np

class tipGoal_to_angles_node3(Node):
    def __init__(self):
        super().__init__('tipGoal_to_angles_node3')
        
        # あるときの関節位置（Unity座標系）からリンク長を計算
        self.boom_pos = np.array([12.14, 1.11, 0])
        self.arm_pos = np.array([12.14, 1.26, 4.8])
        self.bucket_pos = np.array([12.14, 1.26, 7.35])
        self.bucket_end_pos = np.array([12.14, 1.293, 8.48])
        self.L1 = np.linalg.norm(self.arm_pos - self.boom_pos)       # ブームの長さ
        self.L2 = np.linalg.norm(self.bucket_pos - self.arm_pos)     # アームの長さ
        self.L3 = np.linalg.norm(self.bucket_end_pos - self.bucket_pos) # バケットの長さ
        
        # 初期化
        self.swing_joint = None
        self.goal_dir = 0
        self.declare_parameter('machine_name', 'zx120')
        machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        
        # sub and pub
        self.goal_position_sub = self.create_subscription(Point, '/goal_position', self.goal_position_callback, 10) # 目標位置(boomの付け根が原点)
        self.bucket_end_angle_sub = self.create_subscription(Float64, '/bucket_end_angle', self.bucket_end_angle_callback, 10) # バケットの角度
        self.joint_states_sub = self.create_subscription(JointState, '/zx120/joint_states', self.swing_joint_callback, 10) # 現在のswing角
        self.swing_pub = self.create_publisher(Float64, f'{machine_name}/swing/cmd', 10)
        self.boom_pub = self.create_publisher(Float64, f'{machine_name}/boom/cmd', 10)
        self.arm_pub = self.create_publisher(Float64, f'{machine_name}/arm/cmd', 10)
        self.bucket_pub = self.create_publisher(Float64, f'{machine_name}/bucket/cmd', 10)

        # タイマー
        self.timer = self.create_timer(0.05, self.publish_ik_command)

    def goal_position_callback(self, msg):
        self.goal_position = msg

    def bucket_end_angle_callback(self, msg):
        self.bucket_end_angle = msg
        
    def swing_joint_callback(self, msg: JointState):
        if len(msg.position) > 0:
            self.swing_joint = msg.position[0]

    def publish_ik_command(self):
        try:
            x = self.goal_position.x
            y = self.goal_position.y
            z = self.goal_position.z
            offset_x = self.goal_position.x / mt.cos(self.swing_joint)
            alpha = self.bucket_end_angle.data
            
            # 目標位置への角度を計算してswing角度としてpub
            self.goal_dir = mt.atan2(y, x)
            self.swing_pub.publish(Float64(data=self.goal_dir))
            
            # 逆運動学で各関節の角度を計算する
            A = offset_x - self.L3 * mt.sin(alpha)
            B = z - self.L3 * mt.cos(alpha)
            C = (A ** 2 + B ** 2 + self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1)
            gamma = mt.atan2(A, B)
            
            # 到達範囲外の処理
            r = mt.hypot(A, B)
            if r > self.L1 + self.L2 + self.L3 or r < abs(self.L1 - self.L2):
                self.get_logger().warn("目標点が到達範囲外です。建機を走行させてください。")
                return

            theta0 = [0] * 3
            theta1 = [0] * 3
            
            theta0[0] = gamma + mt.acos(C / mt.sqrt(A ** 2 + B ** 2))
            theta0[1] = mt.atan2(A - self.L1 * mt.sin(theta0[0]), B- self.L1 * mt.cos(theta0[0])) - theta0[0]
            theta0[2] = alpha - theta0[0] - theta0[1]

            theta1[0] = gamma - mt.acos(C / mt.sqrt(A ** 2 + B ** 2))
            theta1[1] = mt.atan2(A - self.L1 * mt.sin(theta1[0]), B - self.L1 * mt.cos(theta1[0])) - theta1[0] 
            theta1[2] = alpha - theta1[0] - theta1[1]

            # 各関節の限界角度に基づいてどちらのthetaを選択するか決める
            if (-70.0 <= theta0[0] <= 0.0 and 10.0 <= theta0[1] <= 135 and -30 <= theta0[2] <= 140):
                angle1 =  theta0[0]- mt.radians(90)
                angle2 = theta0[1]
                angle3 = theta0[2]
            else:
                angle1 = theta1[0] - mt.radians(90)
                angle2 = theta1[1]
                angle3 = theta1[2]

            # Publish
            self.boom_pub.publish(Float64(data=angle1))
            self.arm_pub.publish(Float64(data=angle2))
            self.bucket_pub.publish(Float64(data=angle3))

            self.get_logger().info(
                f"L1={self.L1:.1f}, L2={self.L2:.1f}, L3={self.L3:.1f}, "
                f"Sent IK command to: x={x:.2f}, y={y:.2f}, z={z:.2f}, α={mt.degrees(alpha):.1f}° → "
                f"swing={mt.degrees(self.goal_dir):.1f}, boom={mt.degrees(angle1):.1f}°, arm={mt.degrees(angle2):.1f}°, bucket={mt.degrees(angle3):.1f}°"
            )

        except Exception as e:
            self.get_logger().error(f"IK計算中にエラーが発生: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = tipGoal_to_angles_node3()
    rclpy.spin(node)
    rclpy.shutdown()



