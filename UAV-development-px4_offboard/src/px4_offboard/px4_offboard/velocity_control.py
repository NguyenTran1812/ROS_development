#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleStatus,
    VehicleAttitude,
    VehicleCommand,
)
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool


class OffboardState:
    IDLE = "IDLE"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    LOITER = "LOITER"
    OFFBOARD = "OFFBOARD"


class OffboardControl(Node):

    def __init__(self):
        super().__init__('velocity_control')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos)
        self.create_subscription(Twist, '/offboard_velocity_cmd', self.offboard_velocity_callback, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos)
        self.create_subscription(Bool, '/arm_message', self.arm_message_callback, qos)

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Timers
        self.arm_timer = self.create_timer(0.1, self.arm_timer_callback)
        self.cmd_timer = self.create_timer(0.02, self.cmdloop_callback)

        # State variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_INIT
        self.velocity = Vector3()
        self.yaw = 0.0
        self.trueYaw = 0.0
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.current_state = OffboardState.IDLE
        self.last_state = self.current_state

    def arm_message_callback(self, msg: Bool):
        self.arm_message = msg.data
        self.get_logger().info(f"[MSG] Arm message: {self.arm_message}")

    def arm_timer_callback(self):
        if self.current_state == OffboardState.IDLE:
            if self.flightCheck and self.arm_message:
                self.current_state = OffboardState.ARMING
                self.get_logger().info("→ Arming...")

        elif self.current_state == OffboardState.ARMING:
            if not self.flightCheck:
                self.current_state = OffboardState.IDLE
                self.get_logger().info("✖ Flight check failed. Back to IDLE.")
            elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10:
                self.current_state = OffboardState.TAKEOFF
                self.get_logger().info("→ Takeoff...")
            self.arm()

        elif self.current_state == OffboardState.TAKEOFF:
            if not self.flightCheck:
                self.current_state = OffboardState.IDLE
                self.get_logger().info("✖ Flight check failed. Back to IDLE.")
            elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.current_state = OffboardState.LOITER
                self.get_logger().info("→ Loiter...")
            self.arm()
            self.take_off()

        elif self.current_state == OffboardState.LOITER:
            if not self.flightCheck:
                self.current_state = OffboardState.IDLE
                self.get_logger().info("✖ Flight check failed. Back to IDLE.")
            elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.current_state = OffboardState.OFFBOARD
                self.get_logger().info("→ Switch to OFFBOARD mode.")
            self.arm()

        elif self.current_state == OffboardState.OFFBOARD:
            if not self.flightCheck or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                self.current_state = OffboardState.IDLE
                self.get_logger().info("✖ Offboard state invalid. Back to IDLE.")
            else:
                self.state_offboard()

        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm_message = False

        if self.current_state != self.last_state:
            self.get_logger().info(f"[STATE] {self.current_state}")
            self.last_state = self.current_state

        self.myCnt += 1

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("✔ Arm command sent.")

    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)
        self.get_logger().info("⬆ Takeoff command sent.")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def vehicle_status_callback(self, msg):
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"[NAV] {msg.nav_state}")
        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"[ARM] {msg.arming_state}")
        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"[FAILSAFE] {msg.failsafe}")
        if msg.pre_flight_checks_pass != self.flightCheck:
            self.get_logger().info(f"[FlightCheck] {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def offboard_velocity_callback(self, msg: Twist):
        self.velocity.x = -msg.linear.y
        self.velocity.y = msg.linear.x
        self.velocity.z = -msg.linear.z
        self.yaw = msg.angular.z

    def attitude_callback(self, msg: VehicleAttitude):
        q = msg.q
        self.trueYaw = -np.arctan2(2.0 * (q[3]*q[0] + q[1]*q[2]),
                                   1.0 - 2.0 * (q[0]**2 + q[1]**2))

    def cmdloop_callback(self):
        if self.offboardMode:
            now = int(Clock().now().nanoseconds / 1000)

            # Publish OffboardControlMode
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = now
            offboard_msg.velocity = True
            self.publisher_offboard_mode.publish(offboard_msg)

            # Transform velocity from FLU to world
            cos_yaw, sin_yaw = np.cos(self.trueYaw), np.sin(self.trueYaw)
            vx = self.velocity.x * cos_yaw - self.velocity.y * sin_yaw
            vy = self.velocity.x * sin_yaw + self.velocity.y * cos_yaw

            # Publish TrajectorySetpoint
            traj = TrajectorySetpoint()
            traj.timestamp = now
            traj.velocity = [vx, vy, self.velocity.z]
            traj.position = [float('nan')] * 3
            traj.acceleration = [float('nan')] * 3
            traj.yaw = float('nan')
            traj.yawspeed = self.yaw
            self.publisher_trajectory.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

