import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleOdometry, SensorCombined
from std_msgs.msg import Float32

class RealToSimBridge(Node):
    def __init__(self):
        super().__init__('real_to_sim_bridge')

        self.local_pos_pub = self.create_publisher(VehicleLocalPosition, '/sim/vehicle_local_position', 10)
        self.attitude_pub = self.create_publisher(VehicleAttitude, '/sim/vehicle_attitude', 10)
        self.odometry_pub = self.create_publisher(VehicleOdometry, '/sim/vehicle_odometry', 10)
        self.height_pub = self.create_publisher(Float32, '/sim/vehicle_height', 10)
        self.sensor_pub = self.create_publisher(SensorCombined, '/sim/sensor_combined', 10)

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.relay_local_pos, 10)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.relay_attitude, 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.relay_odometry, 10)
        self.create_subscription(Float32, '/vehicle_height', self.relay_height, 10)
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.relay_sensor_combined, 10)

    def relay_local_pos(self, msg):
        self.get_logger().debug("Relaying local position")
        self.local_pos_pub.publish(msg)

    def relay_attitude(self, msg):
        self.get_logger().debug("Relaying attitude")
        self.attitude_pub.publish(msg)

    def relay_odometry(self, msg):
        self.get_logger().debug("Relaying odometry")
        self.odometry_pub.publish(msg)

    def relay_height(self, msg):
        self.get_logger().debug("Relaying height")
        self.height_pub.publish(msg)

    def relay_sensor_combined(self, msg):
        self.get_logger().debug("Relaying sensor_combined")
        self.sensor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealToSimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

