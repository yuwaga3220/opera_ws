#!/usr/bin/env python3
import math

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


class _ImuDev:
    def __init__(self, instrument, frame_id: str, pub_imu, pub_rpy):
        self.instrument = instrument
        self.frame_id = frame_id
        self.pub_imu = pub_imu
        self.pub_rpy = pub_rpy


class WitmotionNode2(Node):
    def __init__(self):
        super().__init__("witmotion_node2")

        # ===== Parameters (配列で複数台指定) =====
        self.declare_parameter("ports", ["/dev/ttyUSB0", "/dev/ttyUSB1"])
        self.declare_parameter("addresses", [80, 80])  # 0x50=80 が2台ともならこう
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("frame_ids", ["imu0_link", "imu1_link"])
        self.declare_parameter("topic_prefixes", ["imu0", "imu1"])  # imu0/data, imu1/data ...
        self.declare_parameter("rate_hz", 20.0)

        ports = list(self.get_parameter("ports").value)
        addrs = list(self.get_parameter("addresses").value)
        baud = int(self.get_parameter("baudrate").value)
        frame_ids = list(self.get_parameter("frame_ids").value)
        prefixes = list(self.get_parameter("topic_prefixes").value)
        rate_hz = float(self.get_parameter("rate_hz").value)

        n = len(ports)
        if not (len(addrs) == len(frame_ids) == len(prefixes) == n):
            raise ValueError(
                f"Parameter length mismatch: ports={len(ports)}, addresses={len(addrs)}, "
                f"frame_ids={len(frame_ids)}, topic_prefixes={len(prefixes)}"
            )

        self.devs = []

        for i in range(n):
            port = ports[i]
            addr = int(addrs[i])
            frame_id = frame_ids[i]
            prefix = prefixes[i]

            inst = minimalmodbus.Instrument(port, addr)
            inst.serial.baudrate = baud
            inst.serial.bytesize = 8
            inst.serial.parity = serial.PARITY_NONE
            inst.serial.stopbits = 1
            inst.serial.timeout = 0.3
            inst.mode = minimalmodbus.MODE_RTU

            pub_imu = self.create_publisher(Imu, f"{prefix}/data", 10)
            pub_rpy = self.create_publisher(Vector3Stamped, f"{prefix}/rpy_deg", 10)

            self.devs.append(_ImuDev(inst, frame_id, pub_imu, pub_rpy))
            self.get_logger().info(f"IMU[{i}] port={port}, addr={addr}, frame_id={frame_id}, topic={prefix}/data")

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(f"Started witmotion_multi_node: {n} IMUs, rate={rate_hz}Hz")

    def _read_and_publish_one(self, dev: _ImuDev, now_msg):
        # レジスタ61-63（0x3D-0x3F）を Holding register (function 0x03) で読む
        regs = dev.instrument.read_registers(61, 3, functioncode=3)

        roll_i = s16(regs[0])
        pitch_i = s16(regs[1])
        yaw_i = s16(regs[2])

        roll_deg = roll_i / 32768.0 * 180.0
        pitch_deg = pitch_i / 32768.0 * 180.0
        yaw_deg = yaw_i / 32768.0 * 180.0

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        qx, qy, qz, qw = quat_from_euler(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = now_msg
        msg.header.frame_id = dev.frame_id

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0

        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        dev.pub_imu.publish(msg)

        rpy = Vector3Stamped()
        rpy.header.stamp = now_msg
        rpy.header.frame_id = dev.frame_id
        rpy.vector.x = roll_deg
        rpy.vector.y = pitch_deg
        rpy.vector.z = yaw_deg
        dev.pub_rpy.publish(rpy)

    def tick(self):
        now_msg = self.get_clock().now().to_msg()

        for idx, dev in enumerate(self.devs):
            try:
                self._read_and_publish_one(dev, now_msg)
            except Exception as e:
                # 片方が落ちてももう片方は継続
                self.get_logger().warn(f"IMU[{idx}] read failed: {e}")


def main():
    rclpy.init()
    node = WitmotionNode2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
