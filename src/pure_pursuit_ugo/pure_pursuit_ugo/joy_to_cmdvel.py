import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math as mt

class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmdvel')
        
        self.declare_parameter('machine_name', 'zx120')
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.05, self.publish_commands)  # 20Hz

        self.publisher = self.create_publisher(Twist, f'{self.machine_name}/tracks/cmd_vel', 10)
        self.swing_pub = self.create_publisher(Float64, f'{self.machine_name}/swing/cmd', 10)
        self.boom_pub = self.create_publisher(Float64, f'{self.machine_name}/boom/cmd', 10)
        self.arm_pub = self.create_publisher(Float64, f'{self.machine_name}/arm/cmd', 10)
        self.bucket_pub = self.create_publisher(Float64, f'{self.machine_name}/bucket/cmd', 10)

        # 目標角度 [rad]
        self.swing_angle = 0.0
        self.boom_angle = 0.0
        self.arm_angle = 0.0
        self.bucket_angle = 0.0

        self.last_joy = Joy()
        self.get_logger().info("JoyToCmdVel Node Started (with angle holding)")

    def joy_callback(self, msg):
        self.last_joy = msg

    def publish_commands(self):
        msg = self.last_joy
        dt = 0.05  # timer周期
        gain = 0.5  # 1秒で0.5rad進む強さ（調整OK）
    

        # 建機の移動制御（Twist）
        twist = Twist()
        twist.linear.x = msg.axes[1] * 1.5
        twist.angular.z = msg.axes[0] * 1.5
        self.publisher.publish(twist)

        # 各関節の角度を蓄積
        self.swing_angle  += msg.buttons[2] * gain * dt
        self.boom_angle   -= msg.buttons[3] * gain * dt
        self.arm_angle    += msg.axes[6] * gain * dt
        self.bucket_angle += msg.axes[7] * gain * dt
        
        self.swing_angle  -= msg.buttons[1] * gain * dt
        self.boom_angle   += msg.buttons[0] * gain * dt

        # 目標角度をFloat64でPublish
        self.swing_pub.publish(Float64(data=self.swing_angle))
        self.boom_pub.publish(Float64(data=self.boom_angle))
        self.arm_pub.publish(Float64(data=self.arm_angle))
        self.bucket_pub.publish(Float64(data=self.bucket_angle))

        # デバッグ出力（任意）
        self.get_logger().info(f'[CmdVel] x={twist.linear.x:.2f}, z={twist.angular.z:.2f}')
        self.get_logger().info(f'[JointAngles] Swing={mt.degrees(self.swing_angle):.2f}, Boom={mt.degrees(self.boom_angle):.2f}, Arm={mt.degrees(self.arm_angle):.2f}, Bucket={mt.degrees(self.bucket_angle):.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    rclpy.spin(node)
    rclpy.shutdown()
