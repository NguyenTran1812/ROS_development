import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitude
from datetime import datetime
import csv
import pytz
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

def quaternion_to_euler(q):
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw  # All in radians

class IMURPYLogger(Node):
    def __init__(self):
        super().__init__('imu_rpy_logger')
        tz = pytz.timezone('Asia/Ho_Chi_Minh')
        now = datetime.now(tz)
        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.fields = ['t', 'roll_rad', 'pitch_rad', 'yaw_rad']
        self.filename = f"imu_rpy_log_{timestamp}_radian.csv"
        self.csvfile = open(self.filename, 'w', newline='')
        self.writer = csv.DictWriter(self.csvfile, fieldnames=self.fields)
        self.writer.writeheader()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            depth=10
        )
        self.subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )

    def attitude_callback(self, msg):
        q = msg.q  # [w, x, y, z]
        roll, pitch, yaw = quaternion_to_euler(q)
        t = msg.timestamp / 1e6  # Convert from microseconds to seconds
        row = {
            't': f"{t:.3f}",
            'roll_rad': f"{roll:.6f}",
            'pitch_rad': f"{pitch:.6f}",
            'yaw_rad': f"{yaw:.6f}",
        }
        self.writer.writerow(row)
        self.csvfile.flush()
        self.get_logger().info(
            f"t: {t:.3f}, roll: {roll:.4f} rad, pitch: {pitch:.4f} rad, yaw: {yaw:.4f} rad"
        )

    def destroy_node(self):
        self.csvfile.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IMURPYLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()