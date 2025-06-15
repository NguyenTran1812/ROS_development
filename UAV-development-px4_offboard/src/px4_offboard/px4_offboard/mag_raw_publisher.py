import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import SensorMag
from geometry_msgs.msg import Vector3

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
            self.callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Vector3, '/sim/mag_raw', 10)

    def callback(self, msg: SensorMag):
        vec = Vector3()
        vec.x = msg.x
        vec.y = msg.y
        vec.z = msg.z

        self.get_logger().info(f'Mag: x={vec.x:.2f}, y={vec.y:.2f}, z={vec.z:.2f}')
        self.publisher.publish(vec)

def main(args=None):
    rclpy.init(args=args)
    node = MagRawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

