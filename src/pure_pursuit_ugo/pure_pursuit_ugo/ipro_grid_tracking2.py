import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import requests
from requests.auth import HTTPDigestAuth

from pure_pursuit_ugo import ipro_env


class IproGridTracking2(Node):
    """
    改善後仕様：
      JointState -> FKで刃先座標計算 -> 刃先方向からチルト計算
        -> 刃先高さから作業フェーズ(Far/Mid/Near) + target_zoom計算 (CameraTracker2.cs準拠)
        -> (tilt, zoom) に最も近いプリセットを選択して送信
        -> 残差チルトは一度だけ directctrl で微調整 (One Shot)
        -> MID中だけ zoom は微調整可能なので繰り返し送る（レート制限つき）
    """

    # -------------------------
    # Phase
    # -------------------------
    class Phase:
        FAR = "Far"
        MID = "Mid"
        NEAR = "Near"

    def __init__(self):
        super().__init__("ipro_grid_tracking2")

        # ========= 建機パラメータ =========
        self.declare_parameter("boom_length", 4.802)
        self.declare_parameter("arm_length", 2.550)
        self.declare_parameter("grapple_offset_z", -1.19)

        # カメラ相対位置（FK座標系と揃えてね）
        self.declare_parameter("camera_offset_x", 0.5)
        self.declare_parameter("camera_offset_z", 1.5)

        self.L_boom = float(self.get_parameter("boom_length").value)
        self.L_arm = float(self.get_parameter("arm_length").value)
        self.G_z = float(self.get_parameter("grapple_offset_z").value)
        self.cam_x = float(self.get_parameter("camera_offset_x").value)
        self.cam_z = float(self.get_parameter("camera_offset_z").value)

        # ========= 対象物トップ高さ（Unityの ObjectTopY 相当） =========
        self.declare_parameter("object_top_z", 0.74)
        self.object_top_z = float(self.get_parameter("object_top_z").value)
        
        self.boom_root_z0 = 1.35 # ZX120のブーム根元高さ（ワールド座標系）

        # ========= フェーズ境界＆ヒステリシス =========
        self.declare_parameter("near_mid_boundary", 1.4)
        self.declare_parameter("mid_far_boundary", 2.3)
        self.declare_parameter("phase_hysteresis", 0.08)

        self.near_mid_boundary = float(self.get_parameter("near_mid_boundary").value)
        self.mid_far_boundary = float(self.get_parameter("mid_far_boundary").value)
        self.phase_hys = float(self.get_parameter("phase_hysteresis").value)

        # ========= プリセットグリッド（10x6） =========
        self.TILT_MIN = -40.0
        self.TILT_MAX = 5.0
        self.T_STEPS = 10

        self.ZOOM_MIN = 1.0
        self.ZOOM_MAX = 3.0
        self.Z_STEPS = 6

        # tilt/zoom の「どっちを優先するか」重み
        self.declare_parameter("tilt_weight", 1.0)
        self.declare_parameter("zoom_weight", 1.0)
        self.w_tilt = float(self.get_parameter("tilt_weight").value)
        self.w_zoom = float(self.get_parameter("zoom_weight").value)

        # ========= ズーム計算 =========
        self.declare_parameter("mid_zoom_max", 3.5)        # MID下端で到達させたいズーム
        self.declare_parameter("max_zoom_factor", 5.0)     # クリップ上限
        self.declare_parameter("zoom_speed", 5.0)          # smoothing
        self.mid_zoom_max = float(self.get_parameter("mid_zoom_max").value)
        self.max_zoom_factor = float(self.get_parameter("max_zoom_factor").value)
        self.zoom_speed = float(self.get_parameter("zoom_speed").value)

        # ========= i-PRO directctrl でズーム微調整をするか =========
        # ※ directctrl の zoom パラメータ仕様が機種で違う可能性があるので、
        #    まずは enable_zoom_fine=False でプリセットのみで動作確認推奨。
        self.declare_parameter("enable_zoom_fine", False)
        self.declare_parameter("zoom_fine_interval_sec", 0.25)
        self.declare_parameter("zoom_deadband", 0.08)       # これ未満の誤差なら送らない
        self.enable_zoom_fine = bool(self.get_parameter("enable_zoom_fine").value)
        self.zoom_fine_interval = float(self.get_parameter("zoom_fine_interval_sec").value)
        self.zoom_deadband = float(self.get_parameter("zoom_deadband").value)

        # directctrl のズーム指定キー（機種によって zoom/zoomspd などの可能性がある）
        self.DIRECT_ZOOM_PARAM = "zoom"

        # ========= 通信 =========
        ipro_env.load_ipro_dotenv()
        ip = ipro_env.camera_ip()
        user = ipro_env.camera_user()
        pw = ipro_env.camera_password()
        if not user or not pw:
            self.get_logger().error(
                ".env に IPRO_CAMERA_USER / IPRO_CAMERA_PASSWORD を設定してください（.env.example 参照）。"
            )
            raise RuntimeError("Missing IPRO_CAMERA_USER or IPRO_CAMERA_PASSWORD")
        self.ctrl_url = f"http://{ip}/cgi-bin/camctrl"
        self.direct_url = f"http://{ip}/cgi-bin/directctrl"

        self.session = requests.Session()
        self.session.auth = HTTPDigestAuth(user, pw)

        # ========= 状態管理 =========
        self.last_preset_change_time = 0.0

        self.stable_preset_id = -1
        self.processed_preset_id = -1
        self.is_moving = False

        # zoom微調整（MIDで繰り返し送る用）
        self._zoom_fine_inflight = False
        self._last_zoom_fine_time = 0.0

        # ========= フェーズ状態 =========
        self.current_phase = self.Phase.FAR
        self.fixed_zoom_level = 1.0
        self.current_zoom_level = 1.0
        self._last_zoom_update_time = time.time()
        
        # ========= 通信セッション =========
        self.auth = HTTPDigestAuth(user, pw)
        self.session.get(self.ctrl_url, auth=self.auth, timeout=3.0)


        self.sub = self.create_subscription(JointState, "/zx120/joint_states", self.callback, 10)
        self.get_logger().info("IproGridTracking2 (Tilt+Zoom phase tracking) Started.")

    # -------------------------
    # Core: callback
    # -------------------------
    def callback(self, msg: JointState):
        try:
            if "boom_joint" not in msg.name or "arm_joint" not in msg.name:
                return

            # ---- FK ----
            boom_idx = msg.name.index("boom_joint")
            arm_idx = msg.name.index("arm_joint")
            th_boom = -msg.position[boom_idx]
            th_arm = -msg.position[arm_idx]

            boom_x = self.L_boom * math.cos(th_boom)
            boom_z = self.L_boom * math.sin(th_boom)
            total = th_boom + th_arm
            tip_x = boom_x + self.L_arm * math.cos(total)
            tip_z = boom_z + self.L_arm * math.sin(total) + self.G_z

            # ---- カメラから見た相対 ----
            rel_x = tip_x - self.cam_x
            rel_z = tip_z - self.cam_z

            # ---- 刃先方向 -> tilt ----
            current_dist = abs(rel_x)
            target_tilt = math.degrees(math.atan2(rel_z, current_dist))

            # ---- 高さ（対象物トップ基準） -> Phase + target_zoom ----
            height_above_object_top = self.boom_root_z0 + tip_z - self.object_top_z
            
            # self.get_logger().info(f"Tip Z: {tip_z:.3f} m")
            # self.get_logger().info(f"Height above object top: {height_above_object_top:.3f} m")
            
            self._update_phase(height_above_object_top)
            target_zoom = self._update_zoom_target(height_above_object_top)

            # ---- プリセット選択＆移動 ----
            self.check_and_move(target_tilt, target_zoom)

            # ---- MID中のズーム微調整（プリセット後に繰り返し送る） ----
            if self.enable_zoom_fine:
                self.maybe_send_zoom_fine(target_zoom)

        except ValueError:
            pass

    # -------------------------
    # Phase & Zoom
    # -------------------------

    # 高さを元にフェーズ更新・Zoom倍率の保持
    def _update_phase(self, h: float):
        # ヒステリシス遷移ロジック
        next_phase = self.current_phase

        near_to_mid = self.near_mid_boundary + self.phase_hys
        mid_to_near = self.near_mid_boundary - self.phase_hys
        mid_to_far = self.mid_far_boundary + self.phase_hys
        far_to_mid = self.mid_far_boundary - self.phase_hys

        if self.current_phase == self.Phase.FAR:
            if h < far_to_mid:
                next_phase = self.Phase.MID

        elif self.current_phase == self.Phase.MID:
            if h >= mid_to_far:
                next_phase = self.Phase.FAR
            elif h < mid_to_near:
                next_phase = self.Phase.NEAR

        elif self.current_phase == self.Phase.NEAR:
            if h >= near_to_mid:
                next_phase = self.Phase.MID

        if next_phase != self.current_phase:
            prev = self.current_phase
            self.current_phase = next_phase

            # Near固定ズーム：入った瞬間のズームを固定値として保持
            if next_phase == self.Phase.NEAR:
                self.fixed_zoom_level = self.current_zoom_level

            # Near -> Mid に戻ったら fixed から再開
            if prev == self.Phase.NEAR and next_phase == self.Phase.MID:
                self.current_zoom_level = self.fixed_zoom_level

            self.get_logger().info(f"[Phase] {prev} -> {next_phase}")

    # MIDのときのズーム倍率を計算
    def _mid_target_zoom_by_height(self, h: float) -> float:
        if self.mid_far_boundary == self.near_mid_boundary:
            return 1.0
        # MIDのうちのどこにいるか（0.0〜1.0）
        u = (h - self.mid_far_boundary) / (self.near_mid_boundary - self.mid_far_boundary)  # 逆向きOK
        u = max(0.0, min(1.0, u))
        z = (1.0 * (1.0 - u)) + (self.mid_zoom_max * u)
        z = max(1.0, min(self.max_zoom_factor, z))
        return z

    # ズーム倍率を更新する
    def _update_zoom_target(self, h: float) -> float:
        """
        Far: 常に 1.0
        Mid: 高さに応じてズーム（smoothing付き）
        Near: 画角固定（＝ズーム固定）
        """
        now = time.time()
        dt = max(1e-3, now - self._last_zoom_update_time)
        self._last_zoom_update_time = now

        if self.current_phase == self.Phase.FAR:
            self.current_zoom_level = 1.0
            return self.current_zoom_level

        if self.current_phase == self.Phase.NEAR:
            # Nearは固定
            self.current_zoom_level = self.fixed_zoom_level
            return self.current_zoom_level

        # Mid（smoothing）
        target = self._mid_target_zoom_by_height(h)
        alpha = max(0.0, min(1.0, dt * self.zoom_speed))
        self.current_zoom_level = (1.0 - alpha) * self.current_zoom_level + alpha * target
        return self.current_zoom_level

    # -------------------------
    # Preset selection (tilt+zoom)
    # -------------------------
    
    # インデックスから理論値（チルト角度）を計算
    def _theory_tilt(self, t_idx: int) -> float:
        return self.TILT_MIN + (t_idx / (self.T_STEPS - 1)) * (self.TILT_MAX - self.TILT_MIN)

    # インデックスから理論値（ズーム倍率）を計算
    def _theory_zoom(self, z_idx: int) -> float:
        return self.ZOOM_MIN + (z_idx / (self.Z_STEPS - 1)) * (self.ZOOM_MAX - self.ZOOM_MIN)

    # 最適プリセット選択（tilt+zoom両方考慮）
    def _select_best_preset(self, target_tilt: float, target_zoom: float):
        """
        10x6 全探索で (tilt, zoom) が最も近い格子点(t_idx,z_idx)を選ぶ。
        近さは正規化した2乗誤差＋重みで評価。
        """
        t_clamped = max(self.TILT_MIN, min(self.TILT_MAX, target_tilt))
        z_clamped = max(self.ZOOM_MIN, min(self.ZOOM_MAX, target_zoom))

        best = None
        best_cost = float("inf")

        tilt_range = max(1e-6, (self.TILT_MAX - self.TILT_MIN))
        zoom_range = max(1e-6, (self.ZOOM_MAX - self.ZOOM_MIN))

        for t_idx in range(self.T_STEPS):
            pt = self._theory_tilt(t_idx)
            dt = (t_clamped - pt) / tilt_range

            for z_idx in range(self.Z_STEPS):
                pz = self._theory_zoom(z_idx)
                dz = (z_clamped - pz) / zoom_range

                cost = (self.w_tilt * dt * dt) + (self.w_zoom * dz * dz)
                if cost < best_cost:
                    best_cost = cost
                    best = (t_idx, z_idx, pt, pz)

        return best  # (t_idx, z_idx, preset_tilt, preset_zoom)

    def check_and_move(self, target_tilt: float, target_zoom: float):
        now = time.time()
        if self.is_moving:
            return

        # === 1) 最適プリセット選択（tilt+zoom） ===
        t_idx, z_idx, preset_tilt_theory, preset_zoom_theory = self._select_best_preset(target_tilt, target_zoom)

        # ID算出 (ID = チルト行 * 6 + ズーム列 + 1) 
        current_raw_id = (t_idx * self.Z_STEPS) + z_idx + 1

        # === 2) プリセットヒステリシス
        if current_raw_id != self.stable_preset_id:
            if now - self.last_preset_change_time > 0.5:
                self.stable_preset_id = current_raw_id
                self.last_preset_change_time = now
            else:
                return
        else:
            self.last_preset_change_time = now

        # === 3) ワンショット実行（プリセットが変わった時だけ)
        if self.stable_preset_id != self.processed_preset_id:
            diff_tilt = target_tilt - preset_tilt_theory

            self.is_moving = True
            self.processed_preset_id = self.stable_preset_id

            threading.Thread(
                target=self.execute_sequence,
                args=(self.stable_preset_id, diff_tilt),
                daemon=True
            ).start()



    # -------------------------
    # Execute sequence: preset -> one-shot tilt refine
    # -------------------------
    def execute_sequence(self, pid: int, diff_tilt: float):

        try:
            # 1) プリセット呼び出し
            self.session.get(f"{self.ctrl_url}?preset={pid}", auth=self.auth, timeout=2.0)

            # パンチルト＋ズームの移動を少し待つ（必要なら調整）
            time.sleep(1.0)

            # 2) 残差チルトだけワンショット微調整
            cmd_tilt = self.calc_tilt_step(diff_tilt)
            if cmd_tilt != 0:
                self._send_direct(pan=0, tilt=cmd_tilt, zoom=None, timeout=2.0)
                time.sleep(0.25)
                self._send_direct(pan=0, tilt=0, zoom=None, timeout=2.0)

            url = f"{self.ctrl_url}?preset={pid}"
            self.get_logger().info(f"[HTTP] GET {url}")
            resp = self.session.get(url, auth=self.auth, timeout=2.0)
            self.get_logger().info(f"[HTTP] status={resp.status_code} body={resp.text[:80]!r}")


        except Exception as e:
            self.get_logger().error(f"Err: {e}")
        finally:
            self.is_moving = False
        


    # -------------------------
    # Mid zoom fine adjust (repeat)
    # -------------------------
    def maybe_send_zoom_fine(self, target_zoom: float):
        # MIDだけ繰り返し送る（NEAR/FARは固定思想）
        if self.current_phase != self.Phase.MID:
            return

        # プリセット移動中は送らない
        if self.is_moving:
            return

        # レート制限
        now = time.time()
        if now - self._last_zoom_fine_time < self.zoom_fine_interval:
            return
        if self._zoom_fine_inflight:
            return

        # 現在のプリセットの理論ズーム（processed_preset_id を逆算）
        if self.processed_preset_id <= 0:
            return

        pid0 = self.processed_preset_id - 1
        z_idx = pid0 % self.Z_STEPS
        preset_zoom = self._theory_zoom(z_idx)

        diff_zoom = target_zoom - preset_zoom
        if abs(diff_zoom) < self.zoom_deadband:
            return

        cmd_zoom = self.calc_zoom_step(diff_zoom)
        if cmd_zoom == 0:
            return

        self._zoom_fine_inflight = True
        self._last_zoom_fine_time = now

        threading.Thread(
            target=self._execute_zoom_fine_once,
            args=(cmd_zoom,),
            daemon=True
        ).start()

    def _execute_zoom_fine_once(self, cmd_zoom: int):
        try:
            # NOTE: directctrl の zoom 指定仕様が機種依存の可能性あり。
            # ここが動かなかったら DIRECT_ZOOM_PARAM や値の表現を調整してね。
            self._send_direct(pan=0, tilt=0, zoom=cmd_zoom, timeout=0.35)
            time.sleep(0.15)
            self._send_direct(pan=0, tilt=0, zoom=0, timeout=0.35)
        except Exception as e:
            self.get_logger().warn(f"Zoom fine err: {e}")
        finally:
            self._zoom_fine_inflight = False

    # -------------------------
    # Direct control helper
    # -------------------------
    def _send_direct(self, pan: int, tilt: int, zoom: int | None, timeout: float):
        params = {"pan": int(pan), "tilt": int(tilt)}
        if zoom is not None:
            params[self.DIRECT_ZOOM_PARAM] = int(zoom)

        # ★ 送信前に表示
        req = requests.Request("GET", self.direct_url, params=params).prepare()
        self.get_logger().info(f"[HTTP] GET {req.url}")

        resp = self.session.get(self.direct_url, params=params, auth=self.auth, timeout=(2.0, timeout))
        self.get_logger().info(f"[HTTP] status={resp.status_code} body={resp.text[:80]!r}")


    # -------------------------
    # Step mapping
    # -------------------------
    def calc_tilt_step(self, diff_deg: float) -> int:
        # 元コードのステップ計算を少し名前だけ変更（挙動同じ） 
        val = abs(diff_deg)
        sign = 1 if diff_deg > 0 else -1
        if val < 0.5:
            return 0
        if val <= 2.0:
            return int(3 + (val - 0.5) * 2) * sign
        return 8 * sign

    def calc_zoom_step(self, diff_zoom: float) -> int:
        """
        zoom差分 -> directctrl コマンド量（仮）
        ※実機で効き方を見て調整する前提（まずは小さめ）。
        """
        val = abs(diff_zoom)
        sign = 1 if diff_zoom > 0 else -1

        # 例：0.08未満は送らない（deadband側でも止まる）
        if val < 0.08:
            return 0
        if val < 0.15:
            return 1 * sign
        if val < 0.25:
            return 2 * sign
        if val < 0.35:
            return 3 * sign
        return 4 * sign


def main(args=None):
    rclpy.init(args=args)
    node = IproGridTracking2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
