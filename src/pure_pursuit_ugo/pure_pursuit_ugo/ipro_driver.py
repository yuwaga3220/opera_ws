import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import requests
import threading
from requests.auth import HTTPDigestAuth

from pure_pursuit_ugo import ipro_env


class IproDriver(Node):
    def __init__(self):
        super().__init__('ipro_driver')

        ipro_env.load_ipro_dotenv()
        self.ip = ipro_env.camera_ip()
        self.user = ipro_env.camera_user()
        self.pw = ipro_env.camera_password()
        if not self.user or not self.pw:
            self.get_logger().error(
                "i-PRO 認証情報がありません。opera_ws/.env に IPRO_CAMERA_USER / "
                "IPRO_CAMERA_PASSWORD を設定するか、.env.example を参照してください。"
            )
            raise RuntimeError("Missing IPRO_CAMERA_USER or IPRO_CAMERA_PASSWORD")

        self.control_url = f"http://{self.ip}/cgi-bin/directctrl"
        self.rtsp_url = (
            f"rtsp://{self.user}:{self.pw}@{self.ip}/MediaInput/stream_1"
        )

        # 1. 映像配信 (Publisher)
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        # 2. 命令受信 (Subscriber)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # 変換ツール
        self.bridge = CvBridge()

        # 通信セッション
        self.session = requests.Session()
        self.session.auth = HTTPDigestAuth(self.user, self.pw)

        # 状態管理用
        self.is_sending = False
        
        # 【重要】最後に送ったコマンドを覚えておく
        # これを使って「命令が変わった時だけ」送信するようにする
        self.last_pan_cmd = 0
        self.last_tilt_cmd = 0
        self.last_zoom_cmd = 0

        # RTSP映像取得スレッド開始
        self.rtsp_thread = threading.Thread(target=self.rtsp_loop)
        self.rtsp_thread.daemon = True
        self.rtsp_thread.start()

        self.get_logger().info(f'i-PRO Driver Started! User: {self.user}')

    def cmd_vel_callback(self, msg):
        """
        /cmd_vel を受け取ってカメラ用パラメータに変換する
        """
        # --- 1. Pan (左右) ---
        # teleopの 'j'(左) は angular.z > 0 -> pan=-1
        # teleopの 'l'(右) は angular.z < 0 -> pan=1
        target_pan = 0
        if msg.angular.z > 0.1:
            target_pan = -1
        elif msg.angular.z < -0.1:
            target_pan = 1
        
        # --- 2. Tilt (上下) ---
        # teleopでは通常出ませんが、プログラム制御用に実装
        target_tilt = 0
        if msg.linear.y > 0.1:
            target_tilt = -1 # 上
        elif msg.linear.y < -0.1:
            target_tilt = 1 # 下

        # --- 3. Zoom (前後) ---
        # teleopの 'i'(前) は linear.x > 0 -> zoom=1
        # teleopの ','(後) は linear.x < 0 -> zoom=-1
        target_zoom = 0
        if msg.linear.x > 0.1:
            target_zoom = 1
        elif msg.linear.x < -0.1:
            target_zoom = -1

        # --- 送信判定 ---
        # 「今の命令」と「さっき送った命令」が違う場合だけ送信する
        # (停止状態から動き出す時、あるいは動き続けていて止まる時など)
        if (target_pan != self.last_pan_cmd or 
            target_tilt != self.last_tilt_cmd or 
            target_zoom != self.last_zoom_cmd):
            
            # パラメータ作成
            params = {}
            
            # Pan/Tiltがある場合
            if target_pan != 0 or target_tilt != 0:
                params['pan'] = target_pan
                params['tilt'] = target_tilt
            # 停止命令の場合 (以前動いていて、今0になった)
            elif self.last_pan_cmd != 0 or self.last_tilt_cmd != 0:
                params['pan'] = 0
                params['tilt'] = 0

            # Zoomがある場合
            if target_zoom != 0:
                params['zoom'] = target_zoom
            # Zoom停止の場合
            elif self.last_zoom_cmd != 0:
                params['zoom'] = 0

            # 送信実行 (スレッド)
            if params:
                # 既に送信中なら今回はスキップ（DDoS防止）してもいいが、
                # 停止命令は優先したいので、タイムアウト短めで投げる
                threading.Thread(target=self.send_http, args=(params,)).start()

            # 記憶を更新
            self.last_pan_cmd = target_pan
            self.last_tilt_cmd = target_tilt
            self.last_zoom_cmd = target_zoom

    def send_http(self, params):
        if self.is_sending:
            return
            
        self.is_sending = True
        try:
            # タイムアウトは短めに
            self.session.get(self.control_url, params=params, timeout=0.5)
            # self.get_logger().info(f"Sent: {params}") # デバッグ用
        except Exception as e:
            self.get_logger().warn(f"HTTP Failed: {e}")
        finally:
            self.is_sending = False

    def rtsp_loop(self):
        """RTSP映像取得ループ"""
        cap = cv2.VideoCapture(self.rtsp_url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            self.get_logger().error(f"Cannot open RTSP stream: {self.rtsp_url}")
            return

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn("Frame dropped")
                continue

            try:
                # ROSメッセージに変換してPublish
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.publisher_.publish(msg)
            except Exception as e:
                pass
        
        cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = IproDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()