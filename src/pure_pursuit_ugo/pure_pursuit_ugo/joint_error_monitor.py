import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointErrorMonitor(Node):
    def __init__(self):
        super().__init__('joint_error_monitor')

        # --- 設定 ---
        # 監視したいキーワードリスト
        # トピック内の関節名に、この文字列が含まれていればデータを拾います
        self.target_keywords = [
            'swing_joint',
            'boom_swing_joint',
            'boom_joint',
            'arm_joint',
            'grapple_upperbody_joint',
            'grapple_lowerbody_joint',
            'grapple_underfork_joint', # ※名前が違う場合は適宜修正 (folk/forkなど)
            'grapple_upperfork_joint'
        ]

        # データ保持用
        self.cmd_data = {}   # 指令値 {full_name: position}
        self.actual_data = {} # 実測値 {full_name: position}

        # --- Subscriber ---
        self.cmd_sub = self.create_subscription(
            JointState,
            'cat303cr/joint_command',
            self.cmd_callback,
            10
        )

        self.actual_sub = self.create_subscription(
            JointState,
            'cat303cr/joint_states', # 名前空間がついている場合は適宜変更 (例: 'cat303cr/joint_states')
            self.actual_callback,
            10
        )

        # 10Hzで表示更新
        self.create_timer(0.1, self.print_status)
        self.get_logger().info("Joint Error Monitor Started (Fuzzy Match Mode).")

    def cmd_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            self.cmd_data[name] = msg.position[i]

    def actual_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            self.actual_data[name] = msg.position[i]

    def find_value_by_keyword(self, data_dict, keyword):
        """
        辞書のキーの中に keyword を含むものがあれば、その値を返す。
        見つからなければ None を返す。
        """
        for full_name, value in data_dict.items():
            if keyword in full_name:
                return value
        return None

    def print_status(self):
        # 画面クリア
        print("\033[H\033[J", end="") 
        
        print(f"{'JOINT KEYWORD':<30} | {'CMD (deg)':<10} | {'ACT (deg)':<10} | {'ERROR (deg)':<10}")
        print("-" * 75)

        for keyword in self.target_keywords:
            # 部分一致検索で値を取得
            cmd = self.find_value_by_keyword(self.cmd_data, keyword)
            actual = self.find_value_by_keyword(self.actual_data, keyword)

            row_str = f"{keyword:<30} | "

            if cmd is not None:
                row_str += f"{math.degrees(cmd):10.2f} | "
            else:
                row_str += f"{'--':^10} | "

            if actual is not None:
                row_str += f"{math.degrees(actual):10.2f} | "
            else:
                row_str += f"{'--':^10} | "

            if cmd is not None and actual is not None:
                diff = cmd - actual
                row_str += f"{math.degrees(diff):10.2f}"
            else:
                row_str += f"{'--':^10}"

            print(row_str)

def main(args=None):
    rclpy.init(args=args)
    node = JointErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()