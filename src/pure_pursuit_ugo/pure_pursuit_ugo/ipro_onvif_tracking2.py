#!/usr/bin/env python3
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

from onvif import ONVIFCamera
import urllib3
from pure_pursuit_ugo import ipro_env

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


def clamp(x: float, a: float, b: float) -> float:
    return max(a, min(b, x))


def linmap(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if abs(x1 - x0) < 1e-9:
        return y0
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)


class IproOnvifTracking2(Node):
    def __init__(self):
        super().__init__("ipro_onvif_tracking2")

        # ====== Camera credentials（.env / 環境変数、または launch で上書き）======
        ipro_env.load_ipro_dotenv()
        self.declare_parameter("camera_ip", ipro_env.camera_ip())
        self.declare_parameter("camera_port", "80")
        self.declare_parameter("camera_user", ipro_env.camera_user())
        self.declare_parameter("camera_pass", ipro_env.camera_password())

        self.ip = str(self.get_parameter("camera_ip").value)
        self.port = int(self.get_parameter("camera_port").value)
        self.user = str(self.get_parameter("camera_user").value)
        self.pw = str(self.get_parameter("camera_pass").value)
        if not self.user or not self.pw:
            self.get_logger().error(
                ".env に IPRO_CAMERA_USER / IPRO_CAMERA_PASSWORD を設定するか、"
                "launch で camera_user / camera_pass を渡してください（.env.example 参照）。"
            )
            raise RuntimeError("Missing camera_user or camera_pass")

        # ====== 建機パラメータ ======(ミニチュア建機用)
        # self.declare_parameter("boom_length", 0.25)
        # self.declare_parameter("arm_length", 0.12)
        # self.declare_parameter("grapple_offset_z", -0.095)

        # self.declare_parameter("camera_offset_x", 0.05)
        # self.declare_parameter("camera_offset_z", 0.06)

        self.declare_parameter("boom_length", 2.45)
        self.declare_parameter("arm_length", 1.30)
        self.declare_parameter("grapple_offset_z", -0.6)

        self.declare_parameter("camera_offset_x", -0.35)
        self.declare_parameter("camera_offset_z", 1.47)
        
        self.L_boom = float(self.get_parameter("boom_length").value)
        self.L_arm = float(self.get_parameter("arm_length").value)
        self.G_z = float(self.get_parameter("grapple_offset_z").value)
        self.cam_x = float(self.get_parameter("camera_offset_x").value)
        self.cam_z = float(self.get_parameter("camera_offset_z").value)

        # ====== IMU角度入力（deg）の符号・オフセット調整 ======
        self.declare_parameter("boom_deg_sign", 1.0)
        self.declare_parameter("arm_deg_sign", 1.0)
        self.declare_parameter("boom_deg_offset", -15.0)
        self.declare_parameter("arm_deg_offset", 0.0)

        self.boom_deg_sign = float(self.get_parameter("boom_deg_sign").value)
        self.arm_deg_sign = float(self.get_parameter("arm_deg_sign").value)
        self.boom_deg_offset = float(self.get_parameter("boom_deg_offset").value)
        self.arm_deg_offset = float(self.get_parameter("arm_deg_offset").value)

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
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("kp_pos", 0.4)
        self.declare_parameter("max_step_pos", 0.08)
        self.declare_parameter("deadband_pos", 0.04)
        self.declare_parameter("send_stop_on_deadband", True)

        self.rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.kp = float(self.get_parameter("kp_pos").value)
        self.max_step = float(self.get_parameter("max_step_pos").value)
        self.deadband = float(self.get_parameter("deadband_pos").value)
        self.send_stop_on_deadband = bool(self.get_parameter("send_stop_on_deadband").value)

        # ====== タイムアウト ======
        self.declare_parameter("imu_timeout_sec", 0.5)
        self.imu_timeout_sec = float(self.get_parameter("imu_timeout_sec").value)

        # ====== target smoothing ======
        self.declare_parameter("target_lpf_alpha", 0.2)
        self.target_lpf_alpha = float(self.get_parameter("target_lpf_alpha").value)
        self._target_tilt_filt: Optional[float] = None

        # ====== PTZコマンド間引き ======
        self.declare_parameter("ptz_cmd_interval_sec", 0.25)
        self.ptz_cmd_interval = float(self.get_parameter("ptz_cmd_interval_sec").value)
        self._last_ptz_cmd_t = 0.0

        # ====== 状態 ======
        self._lock = threading.Lock()

        self._boom_deg: Optional[float] = None
        self._arm_deg: Optional[float] = None
        self._last_boom_t = 0.0
        self._last_arm_t = 0.0

        self._target_tilt_deg: Optional[float] = None
        self._target_pan_deg: float = 0.0

        # ====== ONVIF 初期化 ======
        self._connect_onvif()

        # ====== ROS I/O ======
        self.sub_boom = self.create_subscription(Vector3Stamped, "/imu0/rpy_deg", self.cb_imu0, 10)
        self.sub_arm = self.create_subscription(Vector3Stamped, "/imu1/rpy_deg", self.cb_imu1, 10)
        self.timer = self.create_timer(1.0 / max(1e-3, self.rate_hz), self.control_tick)

        self.get_logger().info("IproOnvifTracking2 started (PT only). Sub: /imu0/rpy_deg, /imu1/rpy_deg")

    # -------------------------
    # ONVIF connect
    # -------------------------
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
    # IMU callbacks
    # -------------------------
    def cb_imu0(self, msg: Vector3Stamped):
        boom_deg = self.boom_deg_sign * (float(msg.vector.x) + self.boom_deg_offset)
        with self._lock:
            self._boom_deg = boom_deg
            self._last_boom_t = time.time()
        self._recompute_targets_if_ready()

    def cb_imu1(self, msg: Vector3Stamped):
        arm_deg = self.arm_deg_sign * (float(msg.vector.x) + self.arm_deg_offset)
        with self._lock:
            self._arm_deg = arm_deg
            self._last_arm_t = time.time()
        self._recompute_targets_if_ready()

    def _recompute_targets_if_ready(self):
        with self._lock:
            if self._boom_deg is None or self._arm_deg is None:
                return
            boom_deg = float(self._boom_deg)
            arm_deg = float(self._arm_deg)

        th_boom_abs = math.radians(boom_deg)
        th_arm_abs = math.radians(arm_deg)

        boom_x = self.L_boom * math.cos(th_boom_abs)
        boom_z = self.L_boom * math.sin(th_boom_abs)

        tip_x = boom_x + self.L_arm * math.cos(th_arm_abs)
        tip_z = boom_z + self.L_arm * math.sin(th_arm_abs) + self.G_z

        rel_x = tip_x - self.cam_x
        rel_z = tip_z - self.cam_z

        current_dist = abs(rel_x)
        target_tilt_deg = math.degrees(math.atan2(rel_z, current_dist))

        with self._lock:
            raw = float(target_tilt_deg)
            if self._target_tilt_filt is None:
                self._target_tilt_filt = raw
            else:
                a = self.target_lpf_alpha
                self._target_tilt_filt = self._target_tilt_filt + a * (raw - self._target_tilt_filt)

            self._target_tilt_deg = float(self._target_tilt_filt)
            self._target_pan_deg = 0.0

    # -------------------------
    # PT: ONVIF RelativeMove
    # -------------------------
    def _pt_control_tick(self, target_pan_deg: float, target_tilt_deg: float):
        now = time.time()
        if (now - self._last_ptz_cmd_t) < self.ptz_cmd_interval:
            return

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
        self._last_ptz_cmd_t = time.time()

    # -------------------------
    # Combined tick
    # -------------------------
    def control_tick(self):
        now = time.time()
        with self._lock:
            if (now - self._last_boom_t) > self.imu_timeout_sec or (now - self._last_arm_t) > self.imu_timeout_sec:
                return
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
    node = IproOnvifTracking2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
