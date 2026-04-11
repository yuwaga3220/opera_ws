#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

import minimalmodbus
import serial


def s16(x: int) -> int:
    return x - 65536 if x > 32767 else x


def quat_from_euler(roll: float, pitch: float, yaw: float):
    """
    roll,pitch,yaw [rad] -> (x,y,z,w)
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class WitmotionNode(Node):
    def __init__(self):
        super().__init__('witmotion_node')

        # ===== Parameters =====
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('address', 80)          # 0x50 = 80
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 20.0)

        port = self.get_parameter('port').value
        addr = int(self.get_parameter('address').value)
        baud = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        # ===== Modbus instrument =====
        self.imu = minimalmodbus.Instrument(port, addr)
        self.imu.serial.baudrate = baud
        self.imu.serial.bytesize = 8
        self.imu.serial.parity = serial.PARITY_NONE
        self.imu.serial.stopbits = 1
        self.imu.serial.timeout = 0.3
        self.imu.mode = minimalmodbus.MODE_RTU

        # Publishers
        self.pub_imu = self.create_publisher(Imu, 'imu/data', 10)
        self.pub_rpy = self.create_publisher(Vector3Stamped, 'imu/rpy_deg', 10)

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"Started witmotion_imu: port={port}, addr={addr}, baud={baud}, rate={rate_hz}Hz"
        )

    def tick(self):
        try:
            # レジスタ61-63（0x3D-0x3F）を Holding register (function 0x03) で読む
            regs = self.imu.read_registers(61, 3, functioncode=3)

            roll_i = s16(regs[0])
            pitch_i = s16(regs[1])
            yaw_i = s16(regs[2])

            # WitMotion系でよくある換算: int16 / 32768 * 180 [deg]
            roll_deg = roll_i / 32768.0 * 180.0
            pitch_deg = pitch_i / 32768.0 * 180.0
            yaw_deg = yaw_i / 32768.0 * 180.0

            roll = math.radians(roll_deg)
            pitch = math.radians(pitch_deg)
            yaw = math.radians(yaw_deg)

            qx, qy, qz, qw = quat_from_euler(roll, pitch, yaw)

            now = self.get_clock().now().to_msg()

            # sensor_msgs/Imu
            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            # 今は角速度・加速度が未実装なので 0（必要なら後で拡張）
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = 0.0

            # 共分散：不明なら -1 で「未提供」を表すのがよくある
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

            self.pub_imu.publish(msg)

            # RPYデバッグ用（度）
            rpy = Vector3Stamped()
            rpy.header.stamp = now
            rpy.header.frame_id = self.frame_id
            rpy.vector.x = roll_deg
            rpy.vector.y = pitch_deg
            rpy.vector.z = yaw_deg
            self.pub_rpy.publish(rpy)

        except Exception as e:
            self.get_logger().warn(f"Read failed: {e}")


def main():
    rclpy.init()
    node = WitmotionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
