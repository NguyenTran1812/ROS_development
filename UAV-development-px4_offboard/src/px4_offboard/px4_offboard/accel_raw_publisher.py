import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleAcceleration
from geometry_msgs.msg import Vector3

class AccelRawPublisher(Node):
    def __init__(self):
        super().__init__('accel_raw_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            VehicleAcceleration,
            '/fmu/out/vehicle_acceleration',
            self.callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Vector3, '/sim/accel_raw', 10)

    def callback(self, msg: VehicleAcceleration):
        vec = Vector3()
        vec.x = msg.xyz[0]
        vec.y = msg.xyz[1]
        vec.z = msg.xyz[2]

        self.get_logger().info(f'Accel: x={vec.x:.2f}, y={vec.y:.2f}, z={vec.z:.2f}')
        self.publisher.publish(vec)

def main(args=None):
    rclpy.init(args=args)
    node = AccelRawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

