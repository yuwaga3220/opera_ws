import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState # JointStateをインポート

class ControllCat2(Node):
    def __init__(self):
        super().__init__('ControllCat2')
        
        self.declare_parameter('machine_name', 'cat303cr')
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.05, self.publish_commands)

        # PublisherをJointState型1つにまとめる
        self.joint_pub = self.create_publisher(JointState, f'{self.machine_name}/joint_command', 10)

        # 関節名を定義しておく
        self.joint_names = [
            'swing_joint', 'boom_swing_joint', 'boom_joint', 'arm_joint', 
            'grapple_joint', 'upperfolk_joint', 'underfolk_joint', 'blade_joint'
        ]

        # 目標角度をリストで管理
        self.joint_positions = [0.0] * len(self.joint_names)

        self.last_joy = Joy()
        self.get_logger().info("ControllCat2Node (JointState version) Started")

    def joy_callback(self, msg):
        self.last_joy = msg

    def publish_commands(self):
        if not self.last_joy.axes: # joyがまだ来ていない場合は何もしない
            return

        msg = self.last_joy
        dt = 0.05
        gain = 0.5

        # 各関節の角度を蓄積 (インデックスで管理)
        self.joint_positions[0] += msg.axes[6] * gain * dt      # swing
        self.joint_positions[1] += msg.axes[0] * gain * dt      # boom_swing
        self.joint_positions[2] += msg.axes[7] * gain * dt      # boom
        self.joint_positions[3] += msg.axes[1] * gain * dt      # arm
        self.joint_positions[4] += (msg.buttons[1] - msg.buttons[2]) * gain * dt # grapple
        self.joint_positions[5] += (msg.buttons[3] - msg.buttons[0]) * gain * dt # upperfolk
        self.joint_positions[6]  = -self.joint_positions[5] # underfolk
        self.joint_positions[7] += (msg.buttons[6] - msg.buttons[7]) * gain * dt # blade

        # JointStateメッセージを作成してPublish
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        
        self.joint_pub.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllCat2()
    rclpy.spin(node)
    rclpy.shutdown()