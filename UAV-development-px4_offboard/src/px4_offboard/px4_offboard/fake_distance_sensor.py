import rclpy
from rclpy.node import Node
from px4_msgs.msg import DistanceSensor

class FakeDistanceSensorNode(Node):
    def __init__(self):
        super().__init__('fake_distance_sensor')
        self.publisher_ = self.create_publisher(DistanceSensor, '/fmu/in/distance_sensor', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = DistanceSensor()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # microseconds
        msg.device_id = 1234
        msg.min_distance = 0.02
        msg.max_distance = 4.0
        msg.current_distance = 1.23
        msg.variance = 0.01
        msg.signal_quality = 100
        msg.type = DistanceSensor.MAV_DISTANCE_SENSOR_LASER  # type laser = 0
        msg.h_fov = 0.5
        msg.v_fov = 0.5
        msg.q = [0.0, 0.0, 0.0, 0.0]  # quaternion orientation (optional)
        msg.orientation = DistanceSensor.ROTATION_FORWARD_FACING  # 0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing fake distance: {msg.current_distance} m')

def main(args=None):
    rclpy.init(args=args)
    node = FakeDistanceSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

