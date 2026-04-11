# pure_pursuit_ugo/follow_point.py
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

def yaw_from_quaternion(q):
    import math
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('follow_line_node')

        # パラメータ
        self.odom = Pose2D()
        self.max_linear_vel = 0.6
        self.max_angular_vel = 1.0
        self.goal = [40, -10]
        self.goal_check_dist = 4.0
        self.k_eta = 1.0
        self.k_omega = 1.5
        
        # 初期位置姿勢が原点x軸方向ではないのでここで初期化パラメータを設定する
        # Unityからの初期位置と初期向き (例: Unityの位置 (x, z), 向きのyaw)
        self.initial_x = 0.0  # Unityのx -> ROSのx
        self.initial_y = 0.0   # Unityのz -> ROSのy
        self.initial_yaw = 0 * math.pi / 180 # Unityの向き（例：0rad）

        # オドメトリの初期化
        self.odom.x = self.initial_x
        self.odom.y = self.initial_y
        self.odom.theta = self.initial_yaw

        # Subscriber / Publisher
        self.create_subscription(PoseStamped, 'zx120/base_link/pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'zx120/tracks/cmd_vel', 10)

        # Timer
        self.create_timer(0.05, self.main_loop)
        self.get_logger().info("Pure Pursuit Node started.")


    def pose_callback(self, msg: PoseStamped):
        # Unity側world座標からの真値位置を直接使う
        self.odom.x = msg.pose.position.x
        self.odom.y = msg.pose.position.y  # UnityのZ軸→ROSのY軸（UnityとROSの軸の違いに注意）

        yaw = yaw_from_quaternion(msg.pose.orientation)
        self.odom.theta = yaw


    def main_loop(self):
        cmd = Twist()
        
        goal_dist = math.hypot(self.goal[0] - self.odom.x, self.goal[1] - self.odom.y)
        goal_angle	= math.atan2(self.goal[1] - self.odom.y, self.goal[0] - self.odom.x) - self.odom.theta
        
        if(goal_angle > math.pi):
            goal_angle -= 2.0*math.pi
        if(goal_angle < -math.pi):
            goal_angle += 2.0*math.pi
            
        if goal_dist < self.goal_check_dist:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Reached goal")
        else:
            # v_x
            cmd.linear.x = self.k_eta * goal_dist
            if(cmd.linear.x > self.max_linear_vel):
                cmd.linear.x = self.max_linear_vel
                
			# omega
            cmd.angular.z = self.k_omega*goal_angle
            if(cmd.angular.z > self.max_angular_vel):
                cmd.angular.z = self.max_angular_vel
            if(cmd.angular.z < -self.max_angular_vel):
                cmd.angular.z = -self.max_angular_vel
            self.get_logger().info(
    f"[x={self.odom.x:.2f}, y={self.odom.y:.2f}, θ={self.odom.theta:.2f}] "
    f"→ goal_dist={goal_dist:.2f}, goal_angle={goal_angle:.2f} "
    f"=> v_x={cmd.linear.x:.2f}, omega={cmd.angular.z:.2f}"
)


        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()
