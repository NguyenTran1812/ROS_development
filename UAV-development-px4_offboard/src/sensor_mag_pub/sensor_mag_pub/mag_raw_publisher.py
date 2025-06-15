import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import SensorMag
from sensor_msgs.msg import MagneticField

class MagRawPublisher(Node):
    def __init__(self):
        super().__init__('mag_raw_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            SensorMag,
            '/fmu/out/sensor_mag',
            self.mag_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(MagneticField, '/sim/mag_raw', 10)

    def mag_callback(self, msg: SensorMag):
        mag_msg = MagneticField()

        # PX4 sensor mag is in gauss, convert to tesla (1 G = 1e-4 T)
        mag_msg.magnetic_field.x = msg.x * 1e-4
        mag_msg.magnetic_field.y = msg.y * 1e-4
        mag_msg.magnetic_field.z = msg.z * 1e-4

        # Optionally: unknown covariance
        mag_msg.magnetic_field_covariance[0] = -1.0

        self.get_logger().info(
            f'MAG Field[x={mag_msg.magnetic_field.x:.4f} T, y={mag_msg.magnetic_field.y:.4f} T, z={mag_msg.magnetic_field.z:.4f} T]'
        )

        self.publisher.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MagRawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

