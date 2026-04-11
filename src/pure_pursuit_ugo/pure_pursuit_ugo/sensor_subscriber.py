import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2  # ← point_cloud2 の便利関数
import numpy as np

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zx120/vlp16',
            self.pointcloud_callback,
            10
        )
        self.get_logger().info("Subscribed to /zx120/vlp16")

    def pointcloud_callback(self, msg):
        # 点群をパースして表示（最初の5点だけ例示）
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points)
        # 正面近くでいくらか高さのある点のみ抽出する
        # ROS座標系に変換済みであることに注意
        filtered = [pt for pt in points_list if 2.0 < pt[0] < 8.0 and 0.0 < pt[1] < 3.0 and -3.0 < pt[2] < 6.0]
        self.get_logger().info(f"Filtered {len(filtered)} points within 5m ahead")

        # self.get_logger().info(f"Received {len(points_list)} points")
        for i, point in enumerate(filtered[:5]):
            self.get_logger().info(f"Point {i}: x={point[0]:.2f}, y={point[1]:.2f}, z={point[2]:.2f}")

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
