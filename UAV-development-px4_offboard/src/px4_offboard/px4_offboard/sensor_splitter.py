#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu

from std_msgs.msg import Header

class SensorSplitter(Node):
    def __init__(self):
        super().__init__('sensor_splitter')

        self.sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_callback,
            10
        )

        self.pub_accel = self.create_publisher(Imu, '/sim/accel_raw', 10)
        self.pub_gyro = self.create_publisher(Imu, '/sim/gyro_raw', 10)
    def sensor_callback(self, msg: SensorCombined):
        # Tạo header
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id='base_link')

        # Xuất accel
        accel_msg = Imu()
        accel_msg.header = header
        accel_msg.linear_acceleration.x = msg.accelerometer_m_s2[0]
        accel_msg.linear_acceleration.y = msg.accelerometer_m_s2[1]
        accel_msg.linear_acceleration.z = msg.accelerometer_m_s2[2]
        self.pub_accel.publish(accel_msg)

        # Xuất gyro
        gyro_msg = Imu()
        gyro_msg.header = header
        gyro_msg.angular_velocity.x = msg.gyro_rad[0]
        gyro_msg.angular_velocity.y = msg.gyro_rad[1]
        gyro_msg.angular_velocity.z = msg.gyro_rad[2]
        self.pub_gyro.publish(gyro_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

