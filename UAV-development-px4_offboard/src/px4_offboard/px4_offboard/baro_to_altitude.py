import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import SensorBaro
from std_msgs.msg import Float32
import math

class BaroToAltitude(Node):
    def __init__(self):
        super().__init__('baro_to_altitude')

        self.declare_parameter('sea_level_pressure', 101325.0)  # Pa

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            SensorBaro,
            '/fmu/out/sensor_baro',
            self.baro_callback,
            qos_profile
        )

        self.altitude_pub = self.create_publisher(Float32, '/sim/baro_altitude_raw', 10)

    def baro_callback(self, msg: SensorBaro):
        P = msg.pressure  # Áp suất đo được (Pa)
        P0 = self.get_parameter('sea_level_pressure').get_parameter_value().double_value

        # Công thức độ cao từ áp suất
        altitude = 44330.0 * (1.0 - math.pow(P / P0, 0.1903))

        self.get_logger().info(f'Baro Pressure: {P:.2f} Pa -> Altitude: {altitude:.2f} m')

        alt_msg = Float32()
        alt_msg.data = altitude
        self.altitude_pub.publish(alt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BaroToAltitude()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
