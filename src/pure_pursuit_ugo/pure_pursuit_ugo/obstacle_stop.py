import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')

        # subscribe: LiDAR点群
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zx120/vlp16',
            self.pointcloud_callback,
            10
        )

        # Publish: 障害物検知フラグ
        self.stop_flag_pub = self.create_publisher(
            Bool,
            '/obstacle_stop',
            10
        )

        self.obstacle_detected = False  # 前回の検知状態

        self.get_logger().info("Subscribed to /zx120/vlp16")

    def pointcloud_callback(self, msg):
        # 点群パース
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points)

        # 正面2〜8m, 左右±3m, 高さ0〜6m の範囲でフィルタ
        filtered = [pt for pt in points_list if 2.0 < pt[0] < 8.0 and 0.0 < pt[1] < 3.0 and -3.0 < pt[2] < 6.0]

        if filtered:
            self.get_logger().info(f"⚠️ 障害物検知: {len(filtered)} 点")
            if not self.obstacle_detected:
                self.publish_stop()
                self.obstacle_detected = True
        else:
            if self.obstacle_detected:
                self.get_logger().info("✅ 障害物クリア")
                self.publish_move()
                self.obstacle_detected = False

    def publish_stop(self):
        self.stop_flag_pub.publish(Bool(data=True))
        self.get_logger().warn("🛑 Stopフラグを送信（True）")

    def publish_move(self):
        self.stop_flag_pub.publish(Bool(data=False))
        self.get_logger().info("▶️ Stopフラグを送信（False）")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
