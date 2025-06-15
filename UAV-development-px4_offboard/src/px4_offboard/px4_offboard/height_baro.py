import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class HeightBaroNode(Node):
    def __init__(self):
        super().__init__('height_baro')

        # QoS cho subscriber giống PX4 sensor data: best_effort + keep last 5
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.listener_callback,
            qos_profile)
        
        self.publisher_ = self.create_publisher(Float32, '/baro_altitude_EKF', 10)

    def listener_callback(self, msg: VehicleLocalPosition):
        altitude = msg.z
        self.get_logger().info(f'Altitude (z): {altitude:.3f} m')
        height_msg = Float32()
        height_msg.data = altitude
        self.publisher_.publish(height_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeightBaroNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user (KeyboardInterrupt)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

