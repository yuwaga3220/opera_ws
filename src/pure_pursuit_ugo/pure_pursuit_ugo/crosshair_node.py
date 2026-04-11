import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, JointState
import numpy as np
import cv2
import math
import os


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
    - JointState: boom_joint, arm_joint, bucket_joint を取得
    - 3リンク簡易モデルで関節位置(3D)を計算
    - 仮のカメラで2D投影して画像に描画
    """

    def __init__(self):
        super().__init__('arm_kinematics_overlay')

        # ---- topics
        self.declare_parameter('image_in', '/unity/camera/image/compressed')
        self.declare_parameter('image_out', '/unity/camera/image_overlay/compressed')
        self.declare_parameter('joint_states', '/zx120/joint_states')

        # ---- joint names（安全のため名前で拾う）
        self.declare_parameter('boom_joint_name', 'boom_joint')
        self.declare_parameter('arm_joint_name', 'arm_joint')
        self.declare_parameter('bucket_joint_name', 'bucket_joint')

        # 以前取得した値なので，現在姿勢とは関係ない．
        self.boom_pos = np.array([12.14, 1.11, 0])
        self.arm_pos = np.array([12.14, 1.26, 4.8])
        self.bucket_pos = np.array([12.14, 1.26, 7.35])
        self.bucket_end_pos = np.array([12.14, 1.293, 8.48])
        
        # ---- link lengths [m]
        self.declare_parameter('boom_len', np.linalg.norm(self.arm_pos - self.boom_pos))
        self.declare_parameter('arm_len', np.linalg.norm(self.bucket_pos - self.arm_pos))
        self.declare_parameter('bucket_len', np.linalg.norm(self.bucket_end_pos - self.bucket_pos))

        # ---- drawing
        self.declare_parameter('thickness', 1)
        self.declare_parameter('joint_radius', 3)
        self.declare_parameter('jpeg_quality', 70)

        # ---- camera intrinsics（仮）
        # fx, fy は画角に相当。まずは適当でOK。合わなければ調整。
        self.declare_parameter('fy', 360 / (2 * math.tan(math.radians(100 / 2))))  # 画角100度相当
        
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
        
        self.declare_parameter('u_offset_px', -30.0)
        self.declare_parameter('v_offset_px', 10.0)
        self.declare_parameter('fx_scale', 1.0)
        self.declare_parameter('fy_scale', 1.0)

        # ---- sandbag overlay (ground decal)
        self.declare_parameter('sandbag_image_path', '/home/hoge/opera_ws/src/pure_pursuit_ugo/pure_pursuit_ugo/assets/sandbag.png')
        self.declare_parameter('sandbag_width_m', 0.9)
        self.declare_parameter('sandbag_height_m', 1.10)
        self.declare_parameter('sandbag_scale', 1.6)
        self.declare_parameter('sandbag_alpha', 0.70)


        # ---- joints unit

        self.image_in = self.get_parameter('image_in').value
        self.image_out = self.get_parameter('image_out').value
        self.joint_topic = self.get_parameter('joint_states').value

        self.boom_name = self.get_parameter('boom_joint_name').value
        self.arm_name = self.get_parameter('arm_joint_name').value
        self.bucket_name = self.get_parameter('bucket_joint_name').value

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

        self.sandbag_img_rgba = None
        self.reload_sandbag_image()

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
            th_bucket_rel = float(self.last_joint[self.bucket_name])
        except KeyError as e:
            self.get_logger().warn(f"Joint not found: {e}")
            self.publish_frame(frame, msg)
            return

        # リンク長
        L1 = float(self.get_parameter('boom_len').value)
        L2 = float(self.get_parameter('arm_len').value)
        L3 = float(self.get_parameter('bucket_len').value)

        base = np.array(self.get_parameter('base_pos').value, dtype=np.float64).reshape(3)

        # ---- 先端(p3)だけ計算（2D: X前方, Z上） ----
        # boom: 絶対角, arm/bucket: 相対角
        a1 = -th_boom
        a2 = a1 - th_arm_rel
        a3 = a2 - th_bucket_rel  # bucketも相対角と仮定

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

        fy = float(self.get_parameter('fy').value) * float(self.get_parameter('fy_scale').value)
        fx = fy * (w / h) * float(self.get_parameter('fx_scale').value)  # fxを画面比に合わせる

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

        # +Y下 にするための反転
        # flip_y = np.diag([1.0, -1.0, 1.0])
        # R_wc = flip_y @ R_wc

        Pc = R_wc @ (p3 - cam_pos)
        X, Y, Z = Pc[0], Pc[1], Pc[2]

        # デバッグ表示（Zが負なら見えない）
        cv2.putText(frame, f"Pc_tip=({X:.2f},{Y:.2f},{Z:.2f})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)

        # baseも点として扱う
        p_base = base.copy()

        # --- 4点を投影 ---
        uv_base, Pc0 = self.project_point(p_base, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv1, Pc1 = self.project_point(p1, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv2, Pc2 = self.project_point(p2, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv3, Pc3 = self.project_point(p3, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)

        # ===== 刃先真下（地面：ワールドY=0）を中心に、ワールドX/Z軸に沿った十字を作る =====
        p0 = p3.copy()
        p0[1] = 0.0  # 地面に落とす中心点 (x_tip, 0, z_tip)

        # 十字の"半径" [m]（好みで調整）
        cross_half = 2.00  # 25cm など

        # X軸方向
        p_xm = p0 + np.array([-cross_half, 0.0, 0.0], dtype=np.float64)
        p_xp = p0 + np.array([+cross_half, 0.0, 0.0], dtype=np.float64)

        # Z軸方向
        p_zm = p0 + np.array([0.0, 0.0, -cross_half], dtype=np.float64)
        p_zp = p0 + np.array([0.0, 0.0, +cross_half], dtype=np.float64)

        # デバッグ表示（先端）
        X, Y, Z = Pc3[0], Pc3[1], Pc3[2]
        cv2.putText(frame, f"Pc_tip=({X:.2f},{Y:.2f},{Z:.2f})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)

        uv0, _  = self.project_point(p0,  R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv_xm, _ = self.project_point(p_xm, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv_xp, _ = self.project_point(p_xp, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv_zm, _ = self.project_point(p_zm, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv_zp, _ = self.project_point(p_zp, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)

        # 地面(y=0)に土のうテクスチャを薄く投影
        frame = self.draw_sandbag_on_ground(
            frame=frame,
            center_world=p0,
            R_wc=R_wc,
            cam_pos=cam_pos,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            u_offset=u_offset,
            v_offset=v_offset,
        )

        # ===== 十字レーザ（ワールドX/Z軸沿い）は非表示 =====
        laser_color = (0, 0, 255)  # 赤(BGR)

        # （任意）刃先→照射点のレーザー線
        if uv3 is not None and uv0 is not None:
            cv2.line(frame, uv3, uv0, laser_color, 1, cv2.LINE_AA)

        # --- 棒（リンク）を描く：見えてる点同士だけ繋ぐ ---
        def draw_link(a, b, color=(255, 0, 0)):
            if a is not None and b is not None:
                cv2.line(frame, a, b, color, self.thickness, cv2.LINE_AA)

        draw_link(uv_base, uv1)  # base->boom先
        draw_link(uv1, uv2)  # boom->arm
        draw_link(uv2, uv3)  # arm->bucket先端

        # --- 関節点を描く（円） ---
        def draw_joint(p, color=(0, 0, 255)):
            if p is not None:
                cv2.circle(frame, p, self.joint_radius, color, -1, cv2.LINE_AA)

        draw_joint(uv_base)  # base
        draw_joint(uv1)  # boom joint
        draw_joint(uv2)  # arm joint
        draw_joint(uv3)  # bucket tip

        # 先端のuvも表示
        if uv3 is not None:
            cv2.putText(frame, f"uv_tip=({uv3[0]},{uv3[1]})",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "tip not visible (Z<=0)",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2, cv2.LINE_AA)
            
        self.publish_frame(frame, msg)

    def project_point(self, P_w, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset):
        """ワールド点P_w(3,)をカメラに投影して(u,v)を返す。見えない場合None。"""
        Pc = R_wc @ (P_w - cam_pos)
        X, Y, Z = Pc[0], Pc[1], Pc[2]
        if Z <= 1e-3:
            return None, Pc
        u = fx * (X / Z) + cx + u_offset
        v = -fy * (Y / Z) + cy + v_offset
        return (int(u), int(v)), Pc

    def reload_sandbag_image(self):
        path = str(self.get_parameter('sandbag_image_path').value).strip()
        self.sandbag_img_rgba = None
        if not path:
            self.get_logger().info("sandbag overlay disabled (sandbag_image_path is empty)")
            return
        if not os.path.isfile(path):
            self.get_logger().warn(f"sandbag image not found: {path}")
            return

        img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if img is None:
            self.get_logger().warn(f"failed to read sandbag image: {path}")
            return

        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGRA)
        elif img.shape[2] == 3:
            b, g, r = cv2.split(img)
            a = np.full_like(b, 255)
            img = cv2.merge([b, g, r, a])
        elif img.shape[2] != 4:
            self.get_logger().warn(f"unsupported sandbag image channels: {img.shape}")
            return

        # 要望対応: 画像は上下反転して使用する
        self.sandbag_img_rgba = cv2.flip(img, 0)
        self.get_logger().info(f"sandbag overlay enabled: {path}")

    def draw_sandbag_on_ground(self, frame, center_world, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset):
        if self.sandbag_img_rgba is None:
            return frame

        width_m = float(self.get_parameter('sandbag_width_m').value)
        height_m = float(self.get_parameter('sandbag_height_m').value)
        scale = float(self.get_parameter('sandbag_scale').value)
        alpha_gain = float(self.get_parameter('sandbag_alpha').value)
        alpha_gain = max(0.0, min(1.0, alpha_gain))
        if scale <= 0.0:
            scale = 1.0
        width_m *= scale
        height_m *= scale
        if width_m <= 0.0 or height_m <= 0.0 or alpha_gain <= 0.0:
            return frame

        cx_w, _, cz_w = center_world.tolist()
        hw = width_m * 0.5
        hh = height_m * 0.5

        # 地面に立つように、Y軸方向に高さを持つ面を作る
        # 面の横方向は「中心->カメラ」のXZベクトルに直交する向きにして、
        # なるべく正面を向くようにする（Y軸回りビルボード）。
        to_cam_xz = np.array([cam_pos[0] - cx_w, 0.0, cam_pos[2] - cz_w], dtype=np.float64)
        n = np.linalg.norm(to_cam_xz)
        if n < 1e-9:
            right = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        else:
            to_cam_xz = to_cam_xz / n
            right = np.array([to_cam_xz[2], 0.0, -to_cam_xz[0]], dtype=np.float64)

        c_bottom = np.array([cx_w, 0.0, cz_w], dtype=np.float64)
        c_top = np.array([cx_w, height_m, cz_w], dtype=np.float64)

        p00 = c_bottom - right * hw  # 左下
        p10 = c_bottom + right * hw  # 右下
        p11 = c_top + right * hw     # 右上
        p01 = c_top - right * hw     # 左上

        uv00, _ = self.project_point(p00, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv10, _ = self.project_point(p10, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv11, _ = self.project_point(p11, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        uv01, _ = self.project_point(p01, R_wc, cam_pos, fx, fy, cx, cy, u_offset, v_offset)
        if None in (uv00, uv10, uv11, uv01):
            return frame

        h_img, w_img = self.sandbag_img_rgba.shape[:2]
        src = np.array([[0, 0], [w_img - 1, 0], [w_img - 1, h_img - 1], [0, h_img - 1]], dtype=np.float32)
        dst = np.array([uv00, uv10, uv11, uv01], dtype=np.float32)
        H = cv2.getPerspectiveTransform(src, dst)

        h, w = frame.shape[:2]
        warped_bgr = cv2.warpPerspective(self.sandbag_img_rgba[:, :, :3], H, (w, h), flags=cv2.INTER_LINEAR)
        warped_a = cv2.warpPerspective(self.sandbag_img_rgba[:, :, 3], H, (w, h), flags=cv2.INTER_LINEAR)

        alpha = (warped_a.astype(np.float32) / 255.0) * alpha_gain
        alpha = alpha[:, :, None]
        out = frame.astype(np.float32) * (1.0 - alpha) + warped_bgr.astype(np.float32) * alpha
        return np.clip(out, 0, 255).astype(np.uint8)

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
