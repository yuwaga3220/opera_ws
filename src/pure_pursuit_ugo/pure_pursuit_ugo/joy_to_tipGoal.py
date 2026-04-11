# 刃先の目標位置と角度をジョイコンで動かしてpublishする(縦平面上に制限)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
import math as mt

class joy_to_tipGoal_node(Node):
    def __init__(self):
        super().__init__('joy_to_tipGoal_node')
        
        self.goal_position_publisher = self.create_publisher(Point, '/goal_position', 10) # 刃先の目標位置
        self.bucket_end_angle_publisher = self.create_publisher(Float64, '/bucket_end_angle', 10) #バケットの水平方向の角度
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10) # ジョイコンの入力

        self.timer = self.create_timer(0.05, self.publish_commands)  # 20Hz

        # 刃先の目標位置とバケットの角度の初期値
        self.goal_position = Point()
        self.goal_position.x = 2.0
        self.goal_position.y = 0.0
        self.goal_position.z = 0.0
        self.bucket_end_angle = Float64()
        self.bucket_end_angle.data = mt.radians(120)
        
        self.last_joy = Joy()
        self.get_logger().info("JoyToCmdVel Node Started (with angle holding)")


    def joy_callback(self, msg):
        self.last_joy = msg

    def publish_commands(self):
        msg = self.last_joy
        dt = 0.05  # timer周期
        gain = 0.5  # 1秒で0.5rad進む強さ（調整OK）
        
        # 刃先の目標位置を制御
        self.goal_position.x += msg.axes[3] * gain * dt  # 前

        self.goal_position.z += msg.buttons[3] * gain * dt  # 上
        self.goal_position.z -= msg.buttons[1] * gain * dt  # 下

        # 右スティックの左右（x軸）でバケットの水平方向の角度を制御
        self.bucket_end_angle.data += msg.buttons[2] * gain * dt # 増
        self.bucket_end_angle.data -= msg.buttons[0] * gain * dt # 減

        # publish
        self.goal_position_publisher.publish(self.goal_position)
        self.bucket_end_angle_publisher.publish(self.bucket_end_angle)

        # ログ出力
        self.get_logger().info(f'Goal position updated: x={self.goal_position.x:.1f}, z={self.goal_position.z:.1f}')
        self.get_logger().info(f'Bucket end angle updated: {mt.degrees(self.bucket_end_angle.data):.1f}')
        
def main(args=None):
    rclpy.init(args=args)
    node = joy_to_tipGoal_node()
    rclpy.spin(node)
    rclpy.shutdown()
