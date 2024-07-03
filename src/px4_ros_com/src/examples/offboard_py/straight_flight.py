#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Float32MultiArray
from colorama import Fore
import numpy as np

def generate_linear_trajectory():
    """Generate points in a linear trajectory from 0.01 to 300.1 with a step of 0.01."""
    x = np.arange(0.01, 300.1, 0.01).tolist()
    y = x  # Diagonal trajectory
    points = list(zip(x, y))
    return points

class StraightFlightArucoDetection(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('straight_flight_aruco_detection')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/offboard_control_mode/in', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/trajectory_setpoint/in', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/vehicle_command/in', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/vehicle_local_position/out', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/vehicle_status/out', self.vehicle_status_callback, qos_profile)
        self.aruco_subscriber = self.create_subscription(Float32MultiArray, "/aruco_detected", self.aruco_callback, 10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -2.6
        self.aruco_marker_detected = False
        self.aruco_xy = [0.0, 0.0]
        self.point_index = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def aruco_callback(self, msg):
        """Callback function for aruco_detected topic subscriber."""
        self.aruco_marker_detected = len(msg.data) == 2
        if self.aruco_marker_detected:
            self.aruco_xy = msg.data
            self.get_logger().info(
                f"{Fore.YELLOW}Aruco marker detected at: {self.aruco_xy[0]}, {self.aruco_xy[1]}{Fore.RESET}")

            # If ArUco marker is detected, adjust position to hover above it
            x_correction, y_correction = self.aruco_xy
            x_current, y_current = self.vehicle_local_position.x, self.vehicle_local_position.y
            self.publish_position_setpoint(x_current + x_correction, y_current + y_correction, self.takeoff_height)
            self.get_logger().info(
                f"Correcting position to hover above Aruco marker at {x_current + x_correction}, {y_current + y_correction}")

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0  # Yaw is not used in this example
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        # After 3 iterations, engage offboard mode and arm the vehicle
        if self.offboard_setpoint_counter == 3:
            self.engage_offboard_mode()
            self.arm()

        # If the vehicle is in offboard mode
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if not self.aruco_marker_detected:
                points = generate_linear_trajectory()
                x, y = points[self.point_index]
                self.publish_position_setpoint(x, y, self.takeoff_height)
                self.point_index += 1


        # Increment the offboard setpoint counter until it reaches 11
        if self.offboard_setpoint_counter < 4:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node, straight flight and aruco marker detection')
    rclpy.init(args=args)
    straight_flight_aruco_detection = StraightFlightArucoDetection()
    rclpy.spin(straight_flight_aruco_detection)
    straight_flight_aruco_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
