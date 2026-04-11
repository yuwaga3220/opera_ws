#!/usr/bin/env python3
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import requests
from requests.auth import HTTPDigestAuth
from onvif import ONVIFCamera
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


def clamp(x: float, a: float, b: float) -> float:
    return max(a, min(b, x))


def linmap(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if abs(x1 - x0) < 1e-9:
        return y0
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)


class IproOnvifTracking(Node):
    class Phase:
        FAR = "Far"
        MID = "Mid"
        NEAR = "Near"

    def __init__(self):
        super().__init__("ipro_onvif_tracking_hybrid")

        # ====== Camera credentials ======
        self.declare_parameter("camera_ip", "192.168.11.200")
        self.declare_parameter("camera_port", "80")
        self.declare_parameter("camera_user", "yuwaga3220")
        self.declare_parameter("camera_pass", "Nagaisawapro1")

        self.ip = str(self.get_parameter("camera_ip").value)
        self.port = int(self.get_parameter("camera_port").value)
        self.user = str(self.get_parameter("camera_user").value)
        self.pw = str(self.get_parameter("camera_pass").value)

        # ====== 建機パラメータ ======
        self.declare_parameter("boom_length", 4.802)
        self.declare_parameter("arm_length", 2.550)
        self.declare_parameter("grapple_offset_z", -1.19)

        self.declare_parameter("camera_offset_x", 0.5)
        self.declare_parameter("camera_offset_z", 1.5)

        self.L_boom = float(self.get_parameter("boom_length").value)
        self.L_arm = float(self.get_parameter("arm_length").value)
        self.G_z = float(self.get_parameter("grapple_offset_z").value)
        self.cam_x = float(self.get_parameter("camera_offset_x").value)
        self.cam_z = float(self.get_parameter("camera_offset_z").value)

        self.declare_parameter("object_top_z", 0.74)
        self.object_top_z = float(self.get_parameter("object_top_z").value)
        self.boom_root_z0 = 1.35

        # ====== フェーズ境界＆ヒステリシス ======
        self.declare_parameter("near_mid_boundary", 1.4)
        self.declare_parameter("mid_far_boundary", 2.3)
        self.declare_parameter("phase_hysteresis", 0.08)

        self.near_mid_boundary = float(self.get_parameter("near_mid_boundary").value)
        self.mid_far_boundary = float(self.get_parameter("mid_far_boundary").value)
        self.phase_hys = float(self.get_parameter("phase_hysteresis").value)

        # ====== ONVIF座標へのマッピング（角度deg <-> ONVIF Position） ======
        self.declare_parameter("pan_deg_min", -40.0)
        self.declare_parameter("pan_deg_max", 40.0)
        self.declare_parameter("tilt_deg_min", -45.0)
        self.declare_parameter("tilt_deg_max", 10.0)

        self.pan_deg_min = float(self.get_parameter("pan_deg_min").value)
        self.pan_deg_max = float(self.get_parameter("pan_deg_max").value)
        self.tilt_deg_min = float(self.get_parameter("tilt_deg_min").value)
        self.tilt_deg_max = float(self.get_parameter("tilt_deg_max").value)

        # ====== PT 制御ループ設定 ======
        self.declare_parameter("control_rate_hz", 8.0)
        self.declare_parameter("kp_pos", 0.8)
        self.declare_parameter("max_step_pos", 0.18)
        self.declare_parameter("deadband_pos", 0.01)
        self.declare_parameter("send_stop_on_deadband", True)

        self.rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.kp = float(self.get_parameter("kp_pos").value)
        self.max_step = float(self.get_parameter("max_step_pos").value)
        self.deadband = float(self.get_parameter("deadband_pos").value)
        self.send_stop_on_deadband = bool(self.get_parameter("send_stop_on_deadband").value)

        # ====== Zoom（HTTP directctrl）設定 ======
        self.declare_parameter("zoom_min", 1.0)
        self.declare_parameter("zoom_max", 3.0)
        self.declare_parameter("zoom_step", 0.1)                 # zoom=±1 1回で 0.1変化（実測）
        self.declare_parameter("zoom_http_interval_sec", 0.25)   # 連打防止（段階ズームなので小さめでもOK）
        self.declare_parameter("enable_zoom_down", True)

        self.zoom_min = float(self.get_parameter("zoom_min").value)
        self.zoom_max = float(self.get_parameter("zoom_max").value)
        self.zoom_step = float(self.get_parameter("zoom_step").value)
        self.zoom_http_interval = float(self.get_parameter("zoom_http_interval_sec").value)
        self.enable_zoom_down = bool(self.get_parameter("enable_zoom_down").value)

        # ★ 段階ズーム（Phaseごとの固定倍率）
        self.declare_parameter("zoom_far", 1.0)
        self.declare_parameter("zoom_mid", 1.5)
        self.declare_parameter("zoom_near", 2.5)

        self.zoom_far = clamp(float(self.get_parameter("zoom_far").value), self.zoom_min, self.zoom_max)
        self.zoom_mid = clamp(float(self.get_parameter("zoom_mid").value), self.zoom_min, self.zoom_max)
        self.zoom_near = clamp(float(self.get_parameter("zoom_near").value), self.zoom_min, self.zoom_max)

        # directctrl URL（いま動いてる方に合わせて https のまま）
        self.DIRECTCTRL_URL = f"https://{self.ip}/cgi-bin/directctrl"

        # ====== フェーズ状態 ======
        self.current_phase = self.Phase.FAR

        # ====== 目標（JointState callbackで更新→タイマーで送信） ======
        self._lock = threading.Lock()
        self._target_tilt_deg: Optional[float] = None
        self._target_pan_deg: float = 0.0

        # ====== HTTP Session（Digest） ======
        self.http = requests.Session()
        self.http.auth = HTTPDigestAuth(self.user, self.pw)

        # ズーム推定値（初期1.0想定）
        self.zoom_est = self.zoom_far

        self._last_zoom_http_time = 0.0
        self._zoom_inflight = False
        self._pending_zoom: Optional[float] = None

        # 401連打回避（簡易バックオフ）
        self._zoom_http_disabled_until = 0.0
        self._zoom_401_count = 0

        # ====== ONVIF 初期化 ======
        self._connect_onvif()

        # 起動時にFarズームへ（念のため）
        self._request_zoom_to(self.zoom_far, reason="startup")

        # ====== ROS I/O ======
        self.sub = self.create_subscription(JointState, "/zx120/joint_states", self.cb_joint, 10)
        self.timer = self.create_timer(1.0 / max(1e-3, self.rate_hz), self.control_tick)

        self.get_logger().info("IproOnvifTrackingHybrid started (PT: ONVIF RelativeMove, Z: HTTP directctrl).")

    def _connect_onvif(self):
        self.get_logger().info(f"Connecting ONVIF camera {self.ip}:{self.port} ...")
        self.cam = ONVIFCamera(self.ip, self.port, self.user, self.pw)
        self.media = self.cam.create_media_service()
        self.ptz = self.cam.create_ptz_service()

        profiles = self.media.GetProfiles()
        if not profiles:
            raise RuntimeError("No ONVIF media profiles found.")
        self.profile = profiles[0]
        self.profile_token = self.profile.token

        try:
            cfg_token = self.profile.PTZConfiguration.token
            opts = self.ptz.GetConfigurationOptions({"ConfigurationToken": cfg_token})
            spaces = getattr(opts, "Spaces", None)

            self.rel_pt_supported = bool(getattr(spaces, "RelativePanTiltTranslationSpace", None))
            self.abs_pt_supported = bool(getattr(spaces, "AbsolutePanTiltPositionSpace", None))
            self.abs_pt_space = spaces.AbsolutePanTiltPositionSpace[0] if self.abs_pt_supported else None

            self.get_logger().info(
                f"ONVIF PTZ support: RelativePT={self.rel_pt_supported}, AbsolutePT={self.abs_pt_supported}"
            )
        except Exception as e:
            self.rel_pt_supported = False
            self.abs_pt_supported = False
            self.abs_pt_space = None
            self.get_logger().warn(f"Failed to read PTZ configuration options: {e}")

    # -------------------------
    # JointState -> target compute
    # -------------------------
    def cb_joint(self, msg: JointState):
        try:
            if "boom_joint" not in msg.name or "arm_joint" not in msg.name:
                return

            boom_idx = msg.name.index("boom_joint")
            arm_idx = msg.name.index("arm_joint")
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
            target_tilt_deg = math.degrees(math.atan2(rel_z, current_dist))

            h = self.boom_root_z0 + tip_z - self.object_top_z
            self._update_phase(h)  # ★フェーズ遷移時にズーム要求を出す

            with self._lock:
                self._target_tilt_deg = target_tilt_deg
                self._target_pan_deg = 0.0

        except Exception:
            return

    # -------------------------
    # Phase update (and stage zoom on transition)
    # -------------------------
    def _phase_to_zoom(self, phase: str) -> float:
        if phase == self.Phase.FAR:
            return self.zoom_far
        if phase == self.Phase.MID:
            return self.zoom_mid
        return self.zoom_near

    def _update_phase(self, h: float):
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
            self.get_logger().info(f"[Phase] {prev} -> {next_phase}")

            # ★ 段階ズーム：フェーズが変わった時だけズームを目標へ寄せる
            desired = self._phase_to_zoom(next_phase)
            self._request_zoom_to(desired, reason=f"phase {prev}->{next_phase}")

    # -------------------------
    # PT: ONVIF RelativeMove
    # -------------------------
    def _pt_control_tick(self, target_pan_deg: float, target_tilt_deg: float):
        if not self.rel_pt_supported:
            return

        st = self.ptz.GetStatus({"ProfileToken": self.profile_token})
        cur_pan = float(st.Position.PanTilt.x)
        cur_tilt = float(st.Position.PanTilt.y)

        if self.abs_pt_space:
            pan_min = float(self.abs_pt_space.XRange.Min)
            pan_max = float(self.abs_pt_space.XRange.Max)
            tilt_min = float(self.abs_pt_space.YRange.Min)
            tilt_max = float(self.abs_pt_space.YRange.Max)
        else:
            pan_min, pan_max = -1.0, 1.0
            tilt_min, tilt_max = -1.0, 1.0

        des_pan = linmap(clamp(target_pan_deg, self.pan_deg_min, self.pan_deg_max),
                         self.pan_deg_min, self.pan_deg_max, pan_min, pan_max)
        des_tilt = linmap(clamp(target_tilt_deg, self.tilt_deg_min, self.tilt_deg_max),
                          self.tilt_deg_min, self.tilt_deg_max, tilt_min, tilt_max)

        err_pan = des_pan - cur_pan
        err_tilt = des_tilt - cur_tilt

        if abs(err_pan) < self.deadband and abs(err_tilt) < self.deadband:
            if self.send_stop_on_deadband:
                try:
                    self.ptz.Stop({"ProfileToken": self.profile_token, "PanTilt": True, "Zoom": False})
                except Exception:
                    pass
            return

        dpan = clamp(self.kp * err_pan, -self.max_step, self.max_step)
        dtilt = clamp(self.kp * err_tilt, -self.max_step, self.max_step)

        req = self.ptz.create_type("RelativeMove")
        req.ProfileToken = self.profile_token
        req.Translation = {"PanTilt": {"x": dpan, "y": dtilt}}
        self.ptz.RelativeMove(req)

    # -------------------------
    # Zoom: HTTP directctrl (stage zoom only on phase change)
    # -------------------------
    def _send_zoom_http(self, zoom_cmd: int, timeout: float = 0.8) -> bool:
        # 401連打回避（カメラ側の制限に引っかかりにくくする）
        if time.time() < self._zoom_http_disabled_until:
            return False

        params = {"zoom": int(zoom_cmd)}
        try:
            resp = self.http.get(self.DIRECTCTRL_URL, params=params, timeout=timeout, verify=False)

            if resp.status_code == 204 or (200 <= resp.status_code < 300):
                self._zoom_401_count = 0
                return True

            if resp.status_code == 401:
                self._zoom_401_count += 1
                wait = min(30.0, 2.0 * (2 ** min(4, self._zoom_401_count)))  # 2,4,8,16,30...
                self.get_logger().warn(f"[ZoomHTTP] 401 -> disable zoom for {wait:.1f}s")
                self._zoom_http_disabled_until = time.time() + wait
                return False

            self.get_logger().warn(f"[ZoomHTTP] status={resp.status_code} body={resp.text[:80]!r}")
            return False

        except Exception as e:
            self.get_logger().warn(f"[ZoomHTTP] error: {e}")
            return False

    def _request_zoom_to(self, desired_zoom: float, reason: str = ""):
        desired_zoom = clamp(desired_zoom, self.zoom_min, self.zoom_max)

        # すでに操作中なら、最後の要求だけ保留（段階ズームなのでこれで十分）
        if self._zoom_inflight:
            self._pending_zoom = desired_zoom
            return

        self._zoom_inflight = True
        self._pending_zoom = None
        threading.Thread(
            target=self._zoom_move_to_thread,
            args=(desired_zoom, reason),
            daemon=True
        ).start()

    def _zoom_move_to_thread(self, desired_zoom: float, reason: str):
        try:
            # diffから必要ステップ数を計算して、zoom=±1 を必要回数だけ送る
            diff = desired_zoom - self.zoom_est
            steps = int(round(diff / max(1e-9, self.zoom_step)))

            if steps == 0:
                return

            if steps < 0 and not self.enable_zoom_down:
                self.get_logger().warn("[Zoom] zoom down disabled; skip.")
                return

            direction = 1 if steps > 0 else -1
            n = abs(steps)

            self.get_logger().info(f"[Zoom] move_to {desired_zoom:.2f} (est={self.zoom_est:.2f}) "
                                   f"steps={steps} reason={reason}")

            # 送信は間引きして確実に（段階ズームなのでこれでOK）
            for _ in range(n):
                now = time.time()
                dt = now - self._last_zoom_http_time
                if dt < self.zoom_http_interval:
                    time.sleep(self.zoom_http_interval - dt)

                if not self._send_zoom_http(direction):
                    break

                # 推定更新
                self.zoom_est = clamp(self.zoom_est + direction * self.zoom_step, self.zoom_min, self.zoom_max)
                self._last_zoom_http_time = time.time()

            # もし保留があれば続けて実行（最新の要求だけ）
            if self._pending_zoom is not None:
                next_zoom = self._pending_zoom
                self._pending_zoom = None
                # 直列にもう一回
                self._zoom_inflight = False
                self._request_zoom_to(next_zoom, reason="pending")
                return

        finally:
            self._zoom_inflight = False

    # -------------------------
    # Combined tick
    # -------------------------
    def control_tick(self):
        with self._lock:
            if self._target_tilt_deg is None:
                return
            target_tilt_deg = float(self._target_tilt_deg)
            target_pan_deg = float(self._target_pan_deg)

        try:
            self._pt_control_tick(target_pan_deg, target_tilt_deg)
        except Exception as e:
            self.get_logger().warn(f"control_tick error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IproOnvifTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
