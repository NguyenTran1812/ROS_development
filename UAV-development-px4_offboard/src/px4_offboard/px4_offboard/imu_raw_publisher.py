import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleImu
from sensor_msgs.msg import Imu

class ImuRawPublisher(Node):
    def __init__(self):
        super().__init__('imu_raw_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            VehicleImu,
            '/fmu/out/vehicle_imu',
            self.imu_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Imu, '/sim/imu_raw', 10)

    def imu_callback(self, msg: VehicleImu):
        imu_msg = Imu()

        # Populate angular velocity
        imu_msg.angular_velocity.x = msg.gyro_rad[0]
        imu_msg.angular_velocity.y = msg.gyro_rad[1]
        imu_msg.angular_velocity.z = msg.gyro_rad[2]

        # Populate linear acceleration
        imu_msg.linear_acceleration.x = msg.accel_m_s2[0]
        imu_msg.linear_acceleration.y = msg.accel_m_s2[1]
        imu_msg.linear_acceleration.z = msg.accel_m_s2[2]

        # Optionally: fill orientation and covariances with zero or unknown
        imu_msg.orientation_covariance[0] = -1.0

        self.get_logger().info(
            f'IMU Accel[x={imu_msg.linear_acceleration.x:.2f}, y={imu_msg.linear_acceleration.y:.2f}, z={imu_msg.linear_acceleration.z:.2f}] '
            f'Gyro[x={imu_msg.angular_velocity.x:.2f}, y={imu_msg.angular_velocity.y:.2f}, z={imu_msg.angular_velocity.z:.2f}]'
        )

        self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

