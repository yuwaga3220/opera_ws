import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, JointState
import numpy as np
import cv2
import math


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]], dtype=np.float64)

def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]], dtype=np.float64)

def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]], dtype=np.float64)



class ArmKinematicsOverlay(Node):
    """
    - JointState: boom_joint, arm_joint, grapple_joint を取得
    - 3リンク簡易モデルで関節位置(3D)を計算
    - 仮のカメラで2D投影して画像に描画
    """

    def __init__(self):
        super().__init__('arm_kinematics_overlay')

        # ---- topics
        self.declare_parameter('image_in', '/unity/camera/image/compressed')
        self.declare_parameter('image_out', '/unity/camera/image_overlay/compressed')
        self.declare_parameter('joint_states', '/cat303cr/joint_states')

        # ---- joint names（安全のため名前で拾う）
        self.declare_parameter('boom_joint_name', 'boom_joint')
        self.declare_parameter('arm_joint_name', 'arm_joint')
        self.declare_parameter('grapple_joint_name', 'grapple_joint')

        # 以前取得した値なので，現在姿勢とは関係ない．
        self.boom_pos = np.array([12.14, 1.11, 0])
        self.arm_pos = np.array([12.14, 1.26, 4.8])
        self.grapple_pos = np.array([12.14, 1.26, 7.35])
        self.grapple_end_pos = np.array([12.14, 1.293, 8.48])
        
        # ---- link lengths [m]
        self.declare_parameter('boom_len', np.linalg.norm(self.arm_pos - self.boom_pos))
        self.declare_parameter('arm_len', np.linalg.norm(self.grapple_pos - self.arm_pos))
        self.declare_parameter('grapple_len', np.linalg.norm(self.grapple_end_pos - self.grapple_pos))

        # ---- drawing
        self.declare_parameter('thickness', 3)
        self.declare_parameter('joint_radius', 6)
        self.declare_parameter('jpeg_quality', 70)

        # ---- camera intrinsics（仮）
        # fx, fy は画角に相当。まずは適当でOK。合わなければ調整。
        self.declare_parameter('fx', 700.0)
        self.declare_parameter('fy', 700.0)
        # cx, cy は画像中心にしたいので 0 の場合は毎フレーム画像中心を使う
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)

        # ---- camera extrinsics（仮）
        # ワールド座標でのカメラ位置 [m]
        self.declare_parameter('cam_pos', [11.32 - 0.739 , -0.06 + 3.23, -7.29 + 1.13])  # (x,y,z)
        # カメラの向き：ここでは「ワールド→カメラ」を作るのが面倒なので
        # とりあえず固定回転（必要ならrpyで組む）
        self.declare_parameter('cam_pitch_deg', 30)  # 下向き
        
        self.declare_parameter('cam_yaw_deg', 0.0)    # 左右
        self.declare_parameter('cam_roll_deg', 0.0)


        # ---- base (boom根本) のワールド座標 [m]
        # 正確なリンク位置の座標は取れないので概算値
        self.declare_parameter('base_pos', [11.373, 1.344, -7.08])
        
        self.declare_parameter('u_offset_px', 0.0)
        self.declare_parameter('v_offset_px', -35.0)
        self.declare_parameter('fx_scale', 0.4)
        self.declare_parameter('fy_scale', 0.4)


        # ---- joints unit

        self.image_in = self.get_parameter('image_in').value
        self.image_out = self.get_parameter('image_out').value
        self.joint_topic = self.get_parameter('joint_states').value

        self.boom_name = self.get_parameter('boom_joint_name').value
        self.arm_name = self.get_parameter('arm_joint_name').value
        self.grapple_name = self.get_parameter('grapple_joint_name').value

        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.thickness = int(self.get_parameter('thickness').value)
        self.joint_radius = int(self.get_parameter('joint_radius').value)

        # latest buffers
        self.last_frame = None
        self.last_header = None
        self.last_joint = None  # dict(name->pos)

        self.sub_img = self.create_subscription(CompressedImage, self.image_in, self.cb_img, 10)
        self.sub_js = self.create_subscription(JointState, self.joint_topic, self.cb_js, 50)
        self.pub = self.create_publisher(CompressedImage, self.image_out, 10)

        self.get_logger().info(f"Subscribed image: {self.image_in}")
        self.get_logger().info(f"Subscribed joint: {self.joint_topic}")
        self.get_logger().info(f"Publishing: {self.image_out}")

    #####################################################################################################
    
    def cb_js(self, msg: JointState):
        # name->position の辞書化
        d = {}
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                d[n] = float(msg.position[i])
        self.last_joint = d

    @staticmethod
    def look_at_R_wc(cam_pos: np.ndarray, target: np.ndarray, up_world: np.ndarray) -> np.ndarray:
        """
        world -> camera rotation
        camera coords: +Z forward, +X right, +Y down (OpenCV-like)
        """
        f = target - cam_pos
        fn = np.linalg.norm(f)
        if fn < 1e-9:
            return np.eye(3, dtype=np.float64)
        f = f / fn

        up = up_world / (np.linalg.norm(up_world) + 1e-9)

        r = np.cross(f, up)
        rn = np.linalg.norm(r)
        if rn < 1e-9:
            up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            r = np.cross(f, up)
            r = r / (np.linalg.norm(r) + 1e-9)
        else:
            r = r / rn

        u = np.cross(r, f)

        R_cw = np.stack([r, -u, f], axis=1)  # camera axes in world (x,y,z)
        return R_cw.T                          # world -> camera


    def cb_img(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Failed to decode CompressedImage")
            return

        # jointが無ければそのまま
        if self.last_joint is None:
            self.publish_frame(frame, msg)
            return

        # 角度取得（名前で）
        try:
            th_boom = float(self.last_joint[self.boom_name])
            th_arm_rel = float(self.last_joint[self.arm_name])
            th_grapple_rel = float(self.last_joint[self.grapple_name])
        except KeyError as e:
            self.get_logger().warn(f"Joint not found: {e}")
            self.publish_frame(frame, msg)
            return

        # リンク長
        L1 = float(self.get_parameter('boom_len').value)
        L2 = float(self.get_parameter('arm_len').value)
        L3 = float(self.get_parameter('grapple_len').value)

        base = np.array(self.get_parameter('base_pos').value, dtype=np.float64).reshape(3)

        # ---- 先端(p3)だけ計算（2D: X前方, Z上） ----
        # boom: 絶対角, arm/grapple: 相対角
        a1 = -th_boom
        a2 = a1 - th_arm_rel
        a3 = a2 - th_grapple_rel  # grappleも相対角と仮定

        # 先端(p3)だけ計算（Z:前方, Y:上） ※Xは左右なので固定
        x0, y0, z0 = base.tolist()

        p1 = np.array([x0,
               y0 + L1 * math.sin(a1),
               z0 + L1 * math.cos(a1)], dtype=np.float64)

        p2 = np.array([p1[0],
                    p1[1] + L2 * math.sin(a2),
                    p1[2] + L2 * math.cos(a2)], dtype=np.float64)

        p3 = np.array([p2[0],
                    p2[1] + L3 * math.sin(a3),
                    p2[2] + L3 * math.cos(a3)], dtype=np.float64)



        # ---- 投影（固定カメラ：baseを見る）----
        h, w = frame.shape[:2]

        fx = float(self.get_parameter('fx').value) * float(self.get_parameter('fx_scale').value)
        fy = float(self.get_parameter('fy').value) * float(self.get_parameter('fy_scale').value)

        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)
        if cx == 0.0: cx = w / 2.0
        if cy == 0.0: cy = h / 2.0

        u_offset = float(self.get_parameter('u_offset_px').value)
        v_offset = float(self.get_parameter('v_offset_px').value)

        cam_pos = np.array(self.get_parameter('cam_pos').value, dtype=np.float64).reshape(3)
        yaw   = math.radians(float(self.get_parameter('cam_yaw_deg').value))
        pitch = math.radians(float(self.get_parameter('cam_pitch_deg').value))
        roll  = math.radians(float(self.get_parameter('cam_roll_deg').value))

        # world -> camera を作る（まずは右手系の想定）
        # カメラ座標: +Z前, +X右, +Y下（OpenCV寄せ）にしたいので、Yを反転する行列を最後に噛ませる
        R_world = rot_y(yaw) @ rot_x(pitch) @ rot_z(roll)     # world basis回転（仮）
        R_wc = R_world.T

        # OpenCVっぽく +Y下 に寄せたいなら（任意）
        flip_y = np.diag([1.0, -1.0, 1.0])
        R_wc = flip_y @ R_wc



        Pc = R_wc @ (p3 - cam_pos)
        X, Y, Z = Pc[0], Pc[1], Pc[2]

        # デバッグ表示（Zが負なら見えない）
        cv2.putText(frame, f"Pc_tip=({X:.2f},{Y:.2f},{Z:.2f})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)

        if Z > 1e-3:
            u = fx * (X / Z) + cx + u_offset
            v = -fy * (Y / Z) + cy + v_offset
            tip = (int(u), int(v))
            cv2.circle(frame, tip, self.joint_radius, (0, 0, 255), -1)  # 赤丸
            cv2.putText(frame, f"uv=({tip[0]},{tip[1]})",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "tip not visible (Z<=0)",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2, cv2.LINE_AA)

        self.publish_frame(frame, msg)


    def publish_frame(self, frame, in_msg: CompressedImage):
        ok, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().warn("Failed to encode JPEG")
            return

        out = CompressedImage()
        out.header = in_msg.header
        out.format = "jpeg"
        out.data = enc.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ArmKinematicsOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
