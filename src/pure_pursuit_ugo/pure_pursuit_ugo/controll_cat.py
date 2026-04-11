import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
import math as mt

class ControllCat(Node):
    def __init__(self):
        super().__init__('ControllCat')
        
        self.declare_parameter('machine_name', 'cat303cr')
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_commands)

        self.swing_pub = self.create_publisher(Float64, f'{self.machine_name}/swing/cmd', 10)
        self.boom_swing_pub = self.create_publisher(Float64, f'{self.machine_name}/boom_swing/cmd', 10)
        self.boom_pub = self.create_publisher(Float64, f'{self.machine_name}/boom/cmd', 10)
        self.arm_pub = self.create_publisher(Float64, f'{self.machine_name}/arm/cmd', 10)
        self.grapple_pub = self.create_publisher(Float64, f'{self.machine_name}/grapple/cmd', 10)
        self.folk_pub = self.create_publisher(Float64, f'{self.machine_name}/folk/cmd', 10)
        self.blade_pub = self.create_publisher(Float64, f'{self.machine_name}/blade/cmd', 10)

        # 目標角度 [rad]
        self.swing_angle = 0.0
        self.boom_swing_angle = 0.0
        self.boom_angle = 0.0
        self.arm_angle = 0.0
        self.grapple_angle = 0.0
        self.folk_angle = 0.0
        self.blade_angle = 0.0

        self.last_joy = Joy()
        self.get_logger().info("ControllCatNode Started (with angle holding)")

    def joy_callback(self, msg):
        self.last_joy = msg

    def publish_commands(self):
        msg = self.last_joy
        dt = 0.1  # timer周期
        gain = 0.5  # 1秒で0.5rad進む強さ

        # 各関節の角度を蓄積
        self.swing_angle      += msg.axes[6] * gain * dt
        self.boom_swing_angle += msg.axes[0] * gain * dt
        self.boom_angle       -= msg.axes[7] * gain * dt
        self.arm_angle     += msg.axes[1] * gain * dt
        
        self.grapple_angle += msg.buttons[1] * gain * dt
        self.grapple_angle -= msg.buttons[2] * gain * dt

        self.folk_angle    += msg.buttons[3] * gain * dt
        self.folk_angle    -= msg.buttons[0] * gain * dt
        
        self.blade_angle   += msg.buttons[6] * gain * dt
        self.blade_angle   -= msg.buttons[7] * gain * dt

        # 目標角度をPublish
        self.swing_pub.publish(Float64(data=self.swing_angle))
        self.boom_swing_pub.publish(Float64(data=self.boom_swing_angle))
        self.boom_pub.publish(Float64(data=self.boom_angle))
        self.arm_pub.publish(Float64(data=self.arm_angle))
        self.grapple_pub.publish(Float64(data=self.grapple_angle))
        self.folk_pub.publish(Float64(data=self.folk_angle))
        self.blade_pub.publish(Float64(data=self.blade_angle))

        # デバッグ出力（任意）
        # self.get_logger().info(f'[CmdVel] x={twist.linear.x:.2f}, z={twist.angular.z:.2f}')
        # self.get_logger().info(f'[JointAngles] Swing={mt.degrees(self.swing_angle):.2f}, Boom={mt.degrees(self.boom_angle):.2f}, Arm={mt.degrees(self.arm_angle):.2f}, Bucket={mt.degrees(self.bucket_angle):.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ControllCat()
    rclpy.spin(node)
    rclpy.shutdown()
