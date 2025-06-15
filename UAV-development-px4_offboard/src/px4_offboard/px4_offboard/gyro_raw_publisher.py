import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleAngularVelocity
from geometry_msgs.msg import Vector3

class GyroRawPublisher(Node):
    def __init__(self):
        super().__init__('gyro_raw_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            VehicleAngularVelocity,
            '/fmu/out/vehicle_angular_velocity',
            self.callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Vector3, '/sim/gyro_raw', 10)

    def callback(self, msg: VehicleAngularVelocity):
        vec = Vector3()
        vec.x = msg.xyz[0]
        vec.y = msg.xyz[1]
        vec.z = msg.xyz[2]

        self.get_logger().info(f'Gyro: x={vec.x:.2f}, y={vec.y:.2f}, z={vec.z:.2f}')
        self.publisher.publish(vec)

def main(args=None):
    rclpy.init(args=args)
    node = GyroRawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

