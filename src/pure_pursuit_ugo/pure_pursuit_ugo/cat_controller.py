import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class CatController(Node):
    def __init__(self):
        super().__init__('CatController')
        
        self.declare_parameter('machine_name', 'cat303cr')
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        
        self.timer = self.create_timer(0.05, self.publish_commands)  # 20Hz

        self.publisher = self.create_publisher(Twist, f'{self.machine_name}/tracks/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.last_joy = Joy()
        self.get_logger().info("CatController Node Started (with angle holding)")

    def joy_callback(self, msg):
        self.last_joy = msg

    def publish_commands(self):
        
        msg = self.last_joy
        dt = 0.05  # timer周期
        gain = 0.5  # 1秒で0.5rad進む強さ（調整OK）
        
        if len(msg.axes) < 2:
            self.get_logger().warn(
            f'Received Joy message with insufficient axes (expected 2, got {len(msg.axes)}). '
            'Is the joystick connected?'
            )
            return        # 建機の移動制御（Twist）
        
        twist = Twist()
        twist.linear.x = msg.axes[1] * 1.5
        twist.angular.z = msg.axes[0] * 1.5
        self.publisher.publish(twist)

        # デバッグ出力（任意）
        self.get_logger().info(f'[CmdVel] x={twist.linear.x:.2f}, z={twist.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = CatController()
    rclpy.spin(node)
    rclpy.shutdown()
