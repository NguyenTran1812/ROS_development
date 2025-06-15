import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from px4_msgs.msg import DistanceSensor
from rclpy.qos import QoSProfile, ReliabilityPolicy

class DistanceSensorNode(Node):
    def __init__(self):
        super().__init__('distance_sensor')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.sub = self.create_subscription(
            Range,
            '/UAV/sonar_ros/out',
            self.range_callback,
            qos_profile
        )
        self.pub = self.create_publisher(
            DistanceSensor,
            '/fmu/in/distance_sensor',
            10
        )
        self.last_log_time = self.get_clock().now()

    def range_callback(self, msg):
        ds = DistanceSensor()
        ds.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ds.min_distance = msg.min_range
        ds.max_distance = msg.max_range
        ds.current_distance = msg.range 
        ds.variance = 0.01  # cm^2
        ds.signal_quality = 100
        ds.type = DistanceSensor.MAV_DISTANCE_SENSOR_ULTRASOUND
        ds.orientation = DistanceSensor.ROTATION_DOWNWARD_FACING
        ds.device_id = 3
        ds.h_fov = msg.field_of_view
        ds.v_fov = msg.field_of_view

        self.pub.publish(ds)

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1e9:
            self.get_logger().info(f"Published distance: {msg.range:.2f} m")
            self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

