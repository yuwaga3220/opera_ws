import cv2
import requests
import threading
import time
from requests.auth import HTTPDigestAuth

# ==== 設定 (ここだけ書き換えてください) ====
IP_ADDRESS = '192.168.11.200'
USER = 'yuwaga3220'
PASS = 'Nagaisawapro1'
# ==========================================

# 正解のURLに変更 (directctrl)
CONTROL_URL = f"http://{IP_ADDRESS}/cgi-bin/directctrl"
RTSP_URL = f"rtsp://{USER}:{PASS}@{IP_ADDRESS}/MediaInput/stream_1"

is_sending = False

def send_command_thread(session, params):
    global is_sending
    try:
        # タイムアウトを少し長めに確保
        res = session.get(CONTROL_URL, params=params, timeout=1.0)
        print(f"Sent: {params} | Result: {res.status_code}")
        
        if res.status_code == 403:
            print(">>> 403エラー: 座標が範囲外か、この機能が無効です <<<")
            
    except Exception as e:
        print(f"Failed: {e}")
    finally:
        is_sending = False  # 送信完了フラグを下ろす

def main():
    global current_pan, current_tilt, is_sending
    
    session = requests.Session()
    session.auth = HTTPDigestAuth(USER, PASS)

    print(f"Connecting to: {RTSP_URL}")
    cap = cv2.VideoCapture(RTSP_URL)
    
    if not cap.isOpened():
        print("映像取得エラー")
        return

    # 遅延対策
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print("=== 操作方法 (v3: 安全版) ===")
    print(" [A][D]: パン移動 (座標制限あり)")
    print(" [W][S]: ズーム操作")
    print(" [SPACE]: ズーム停止")
    print(" [Z]: リセット (0,0)")
    print(" [Q]: 終了")
    print("===========================")

    is_moving = False
    is_zooming = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        cv2.imshow('i-PRO Control v3', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        # --- コマンド送信ロジック ---
        
        # すでに通信中なら、新しいキー入力は無視する (DDOS防止)
        if is_sending:
            continue

        target_params = None
        
            
        if is_moving == False: # 動いていないなら
            if key == ord('a'): # 左
                target_params = {'pan': -1, 'tilt': 0}
                is_moving = True
            elif key == ord('d'): # 右
                target_params = {'pan': 1, 'tilt': 0}
                is_moving = True
            elif key == ord('w'): # 上
                target_params = {'pan': 0, 'tilt': -1}
                is_moving = True
            elif key == ord('s'): # 下
                target_params = {'pan': 0, 'tilt': 1}
                is_moving = True
        else: # 動いているならすぐ止める
            target_params = {'pan': 0, 'tilt': 0}
            is_moving = False

        if is_zooming == False:
            if key == ord('x'): # ズームイン
                target_params = {'zoom': 1}
            elif key == ord('c'): # ズームアウト
                target_params = {'zoom': -1}
            elif key == ord(' '):
                target_params = {'pan': 0, 'tilt': 0}
        else: 
            target_params = {'zoom': 0}
            is_zooming = False

        # 送信実行
        if target_params is not None:
            is_sending = True
            threading.Thread(target=send_command_thread, args=(session, target_params)).start()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()