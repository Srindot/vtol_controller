#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class OffboardControl(Node):
    """Node for controlling a VTOL in fixed-wing mode with altitude and airspeed maintenance."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, qos_profile)

        # Configuration parameters
        self.transition_altitude = -30.0  # NED frame (30m above home)
        self.desired_airspeed = 15.0      # m/s
        self.climb_rate = 2.0            # m/s

        # State variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.transition_requested = False
        self.transition_confirmed = False
        self.airspeed_set = False

        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        """Handle vehicle local position updates"""
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        """Handle vehicle status updates"""
        self.vehicle_status = msg
        # Check if transition to fixed-wing is complete
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_FW_POSCTL:
            self.transition_confirmed = True
            self.get_logger().info("Fixed-wing mode confirmed!")

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def transition_to_fixed_wing(self):
        """Transition to fixed-wing mode with safety checks."""
        if (self.vehicle_local_position.z <= self.transition_altitude and 
            self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED and
            not self.transition_requested):
            
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 
                param1=3.0  # MAV_VTOL_STATE_FW
            )
            self.get_logger().info("Transitioning to fixed-wing mode")
            self.transition_requested = True

    def set_airspeed(self):
        """Set target airspeed for fixed-wing mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED,
            param1=0.0,  # 0 = airspeed
            param2=self.desired_airspeed
        )
        self.get_logger().info(f"Airspeed set to {self.desired_airspeed} m/s")
        self.airspeed_set = True

    def publish_offboard_control_mode(self, position=True, velocity=False):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        """Publish altitude setpoint."""
        msg = TrajectorySetpoint()
        
        if self.transition_confirmed:
            # Fixed-wing control (airspeed + altitude)
            msg.position = [float('nan'), float('nan'), self.transition_altitude]
            msg.velocity = [self.desired_airspeed, float('nan'), float('nan')]
        else:
            # Multicopter climb
            msg.position = [0.0, 0.0, self.transition_altitude]
            
        msg.yaw = 0.0  # Maintain current heading
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish vehicle commands."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Main control loop."""
        # Initialization phase
        if self.offboard_setpoint_counter < 10:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
            self.offboard_setpoint_counter += 1
            return

        # Arm and enter offboard mode
        if not self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.arm()
            self.engage_offboard_mode()
            return

        # Pre-transition phase (climb to altitude)
        if not self.transition_requested:
            if self.vehicle_local_position.z > self.transition_altitude:  # Corrected altitude check
                self.publish_trajectory_setpoint()
                self.get_logger().info(f"Current altitude: {self.vehicle_local_position.z}m")
            else:
                self.transition_to_fixed_wing()

        # Post-transition phase (fixed-wing control)
        if self.transition_confirmed:
            if not self.airspeed_set:
                self.set_airspeed()
            
            # Maintain altitude and airspeed
            self.publish_offboard_control_mode(position=True, velocity=True)
            self.publish_trajectory_setpoint()

def main(args=None) -> None:
    print('Starting VTOL fixed-wing altitude/airspeed control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
