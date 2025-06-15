# vehicle_local_subscriber.py
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition

class VLPSub(Node):
    def __init__(self):
        super().__init__('vlp_listener')
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'POS: x={msg.x}, y={msg.y}, z={msg.z}')

def main(args=None):
    rclpy.init(args=args)
    node = VLPSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

