import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
import math
import numpy as np

class GrappleTrackingNode(Node):
    def __init__(self):
        super().__init__('grapple_tracking_node')

        # === パラメータ設定 ===
        # 建機のリンク長
        self.declare_parameter('boom_length', 4.802)
        self.declare_parameter('arm_length', 2.550)
        self.L3 = 1.130
        
        # 刃先オフセット [m] (アーム先端関節から刃先までの距離)
        # ここでは単純化のため、アームの延長線上にL3があると仮定してZ軸オフセットに設定
        self.declare_parameter('grapple_offset_x', 0.0) 
        self.declare_parameter('grapple_offset_y', 0.0)
        self.declare_parameter('grapple_offset_z', -2 * float(self.L3))

        # カメラ位置オフセット [m]
        self.declare_parameter('camera_offset_x', 0.5)
        self.declare_parameter('camera_offset_y', 0.3)
        self.declare_parameter('camera_offset_z', 1.5)

        # パラメータの取得
        self.L_boom = self.get_parameter('boom_length').value
        self.L_arm = self.get_parameter('arm_length').value
        
        # NumPy配列として保存
        self.G_pos = np.array([
            self.get_parameter('grapple_offset_x').value,
            self.get_parameter('grapple_offset_y').value,
            self.get_parameter('grapple_offset_z').value
        ])
        self.cam_pos = np.array([
            self.get_parameter('camera_offset_x').value,
            self.get_parameter('camera_offset_y').value,
            self.get_parameter('camera_offset_z').value
        ])

        # === 通信設定 ===
        self.subscription = self.create_subscription(
            JointState,
            '/zx120/joint_states',
            self.joint_callback,
            10
        )
        self.ptz_pub = self.create_publisher(Vector3, '/camera/ptz_cmd', 10)

        self.get_logger().info('Grapple Tracking Node Started.')

    def joint_callback(self, msg):
        try:
            if 'boom_joint' not in msg.name or 'arm_joint' not in msg.name:
                self.get_logger().warn("Received JointState does not contain required joints.")
                return

            boom_idx = msg.name.index('boom_joint')
            arm_idx = msg.name.index('arm_joint')

            # 角度 [rad]
            theta_boom = -msg.position[boom_idx]
            theta_arm = -msg.position[arm_idx]
            
            self.get_logger().info(f"Received Joint Angles: Boom={theta_boom:.3f}, Arm={theta_arm:.3f}")

            # --- 1. 順運動学 (FK) ---
            
            # ブーム先端
            boom_tip_x = self.L_boom * math.cos(theta_boom)
            boom_tip_z = self.L_boom * math.sin(theta_boom)

            # アーム先端
            total_angle = theta_boom + theta_arm
            arm_tip_x = boom_tip_x + self.L_arm * math.cos(total_angle)
            arm_tip_z = boom_tip_z + self.L_arm * math.sin(total_angle)
            
            g_off_x = self.G_pos[0]
            g_off_z = self.G_pos[2] # L3

            tip_x = arm_tip_x + g_off_x
            tip_z = arm_tip_z + g_off_z 
            tip_y = 0.0 

            target_pos = np.array([tip_x, tip_y, tip_z])

            # --- 2. カメラ座標系への変換 ---
            rel_pos = target_pos - self.cam_pos
            
            # --- 3. パン・チルト・ズームの計算 ---
            
            # パン (Pan) の計算を復活
            pan_rad = math.atan2(rel_pos[1], rel_pos[0])
            
            # チルト (Tilt)
            # 水平距離(奥行き)を使って正確な仰角を出す
            distance_horizontal = math.sqrt(rel_pos[0]**2 + rel_pos[1]**2)
            tilt_rad = math.atan2(rel_pos[2], distance_horizontal)

            # --- 4. メッセージ作成 ---
            cmd_msg = Vector3()
            
            # ROS(rad) -> Unity(deg) 変換
            cmd_msg.x = -math.degrees(pan_rad)
            cmd_msg.y = -math.degrees(tilt_rad)
            cmd_msg.z = 0.0

            self.ptz_pub.publish(cmd_msg)

            # デバッグログ (間引いて表示推奨)
            # self.get_logger().info(f"Tip:({tip_x:.2f}, {tip_z:.2f}) -> PT:({cmd_msg.x:.2f}, {cmd_msg.y:.2f})")

        except ValueError:
            pass
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GrappleTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()