import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import numpy as np
import requests
from requests.auth import HTTPDigestAuth
import threading
import time

# ==== 通信設定 ====
IP_ADDRESS = '192.168.11.200'
USER = 'yuwaga3220'
PASS = 'Nagaisawapro1'
CTRL_URL = f"http://{IP_ADDRESS}/cgi-bin/camctrl"
DIRECT_URL = f"http://{IP_ADDRESS}/cgi-bin/directctrl"

class IproOneShotTiltOnly(Node):
    def __init__(self):
        super().__init__('ipro_oneshot_tilt_only')

        # 建機パラメータ
        self.declare_parameter('boom_length', 4.802)
        self.declare_parameter('arm_length', 2.550)
        self.declare_parameter('grapple_offset_z', -2.26) 
        self.declare_parameter('camera_offset_x', 0.5)
        self.declare_parameter('camera_offset_z', 1.5)

        self.L_boom = self.get_parameter('boom_length').value
        self.L_arm = self.get_parameter('arm_length').value
        self.G_z = self.get_parameter('grapple_offset_z').value
        self.cam_x = self.get_parameter('camera_offset_x').value
        self.cam_z = self.get_parameter('camera_offset_z').value

        # === 10x6 グリッド定義 ===
        self.TILT_MIN = -40.0
        self.TILT_MAX = 5.0
        self.T_STEPS = 10
        
        # ズーム定義（計算には使うが、今回は固定値を入れる）
        self.ZOOM_MIN = 1.0
        self.ZOOM_MAX = 3.0
        self.Z_STEPS = 6

        # 通信設定
        self.session = requests.Session()
        self.session.auth = HTTPDigestAuth(USER, PASS)
        
        # 状態管理
        self.last_preset_change_time = 0
        
        # ヒステリシス制御用
        self.stable_preset_id = -1
        self.processed_preset_id = -1
        self.is_moving = False

        self.sub = self.create_subscription(JointState, '/zx120/joint_states', self.callback, 10)
        self.get_logger().info('Tilt-Only One-Shot Tracking Started.')

    def callback(self, msg):
        try:
            if 'boom_joint' not in msg.name or 'arm_joint' not in msg.name: return

            # FK計算
            boom_idx = msg.name.index('boom_joint')
            arm_idx = msg.name.index('arm_joint')
            th_boom = -msg.position[boom_idx]
            th_arm = -msg.position[arm_idx]

            boom_x = self.L_boom * math.cos(th_boom)
            boom_z = self.L_boom * math.sin(th_boom)
            total = th_boom + th_arm
            tip_x = boom_x + self.L_arm * math.cos(total)
            tip_z = boom_z + self.L_arm * math.sin(total) + self.G_z

            rel_x = tip_x - self.cam_x
            rel_z = tip_z - self.cam_z
            
            current_dist = abs(rel_x)
            target_tilt = math.degrees(math.atan2(rel_z, current_dist))
            
            # 【変更点】ズームは計算せず、強制的に 1.0 (最小) に固定
            target_zoom = 1.0

            self.check_and_move(target_tilt, target_zoom)

        except ValueError: pass

    def check_and_move(self, target_tilt, target_zoom):
        now = time.time()
        
        if self.is_moving:
            return

        # === 1. ID計算 ===
        t_clamped = max(self.TILT_MIN, min(self.TILT_MAX, target_tilt))
        t_idx = int(round( (t_clamped - self.TILT_MIN) / (self.TILT_MAX - self.TILT_MIN) * (self.T_STEPS - 1) ))

        z_clamped = max(self.ZOOM_MIN, min(self.ZOOM_MAX, target_zoom))
        z_idx = int(round( (z_clamped - self.ZOOM_MIN) / (self.ZOOM_MAX - self.ZOOM_MIN) * (self.Z_STEPS - 1) ))

        # ID算出 (ID = チルト行 * 6 + ズーム列 + 1)
        # ズームが1.0固定なので z_idx は常に 0 になり、
        # IDは 1, 7, 13, 19, 25... (各チルト角のズームx1.0版) が選ばれます。
        current_raw_id = (t_idx * self.Z_STEPS) + z_idx + 1

        # === 2. ヒステリシス制御 (IDの確定) ===
        if current_raw_id != self.stable_preset_id:
            if now - self.last_preset_change_time > 0.5:
                self.stable_preset_id = current_raw_id
                self.last_preset_change_time = now
            else:
                return
        else:
            self.last_preset_change_time = now

        # === 3. ワンショット実行判定 ===
        if self.stable_preset_id != self.processed_preset_id:
            
            # 微調整量の計算
            preset_tilt_theory = self.TILT_MIN + (t_idx / (self.T_STEPS - 1)) * (self.TILT_MAX - self.TILT_MIN)
            diff_tilt = target_tilt - preset_tilt_theory

            self.is_moving = True
            self.processed_preset_id = self.stable_preset_id
            
            threading.Thread(
                target=self.execute_sequence,
                args=(self.stable_preset_id, diff_tilt)
            ).start()

    def execute_sequence(self, pid, diff):
        try:
            # 1. プリセット呼び出し
            self.session.get(f"{CTRL_URL}?preset={pid}", timeout=0.5)
            
            # ズーム移動が無い分、待機時間は短くて済むはずですが
            # 念のためパンチルト移動完了まで待つ
            time.sleep(1.0) 

            # 2. 微調整 (One Shot)
            cmd_tilt = self.calc_step(diff)
            if cmd_tilt != 0:
                self.session.get(f"{DIRECT_URL}?pan=0&tilt={cmd_tilt}", timeout=0.3)
                time.sleep(0.3) 
                
                # 3. 停止
                self.session.get(f"{DIRECT_URL}?pan=0&tilt=0", timeout=0.3)

        except Exception as e:
            self.get_logger().error(f"Err: {e}")
        finally:
            self.is_moving = False

    def calc_step(self, diff):
        val = abs(diff)
        sign = 1 if diff > 0 else -1
        if val < 0.5: return 0
        if val <= 2.0: return int(3 + (val-0.5)*2) * sign
        else: return 8 * sign

def main(args=None):
    rclpy.init(args=args)
    node = IproOneShotTiltOnly()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()