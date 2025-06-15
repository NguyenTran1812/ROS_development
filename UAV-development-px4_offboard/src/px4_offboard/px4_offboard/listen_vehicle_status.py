import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus

class VehicleStatusListener(Node):
    def __init__(self):
        super().__init__('vehicle_status_listener')
        self.subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        print("Vehicle Status received:")
        print(f"  nav_state: {msg.nav_state}")
        print(f"  armed: {msg.armed}")
        print(f"  timestamp: {msg.timestamp}")

def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatusListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

