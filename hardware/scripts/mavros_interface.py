#!/usr/bin/env python3
"""
MAVROS Interface for Hardware Deployment
Provides abstraction layer between mission control and MAVROS/Pixhawk.

This module bridges the gap between Gazebo simulation velocity control
and real hardware using MAVROS with PX4/ArduPilot.

Author: Drone Project Team
Version: 2.0.0
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, BatteryState, NavSatFix, Imu
from mavros_msgs.msg import State, ExtendedState, HomePosition, WaypointList
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome

# Import common utilities
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from optimized.mission_common import (
    DroneState, VelocityCommand, SafetyLimits, RateLimiter,
    clamp, angle_wrap, quat_to_yaw, distance_2d, MissionPhase
)


class FlightMode(Enum):
    """PX4 flight modes."""
    MANUAL = "MANUAL"
    ACRO = "ACRO"
    OFFBOARD = "OFFBOARD"
    STABILIZED = "STABILIZED"
    RATTITUDE = "RATTITUDE"
    ALTCTL = "ALTCTL"  # Altitude control
    POSCTL = "POSCTL"  # Position control
    AUTO.LOITER = "AUTO.LOITER"
    AUTO.RTL = "AUTO.RTL"
    AUTO.LAND = "AUTO.LAND"
    AUTO.MISSION = "AUTO.MISSION"


class ArmState(Enum):
    """Vehicle arm states."""
    DISARMED = 0
    ARMED = 1


@dataclass
class HardwareConfig:
    """Hardware-specific configuration."""
    # MAVROS namespace
    mavros_ns: str = "/mavros"
    
    # Frame IDs
    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    
    # Control parameters
    offboard_setpoint_rate: float = 20.0  # Hz
    offboard_timeout: float = 0.5  # seconds
    
    # Safety parameters
    min_gps_fix_type: int = 3  # 3D fix required
    max_position_error_for_arm: float = 2.0  # meters
    
    # Takeoff parameters
    default_takeoff_altitude: float = 4.0
    takeoff_speed: float = 1.0
    
    # RTL parameters
    rtl_altitude: float = 10.0
    
    # Sensor timeouts
    battery_timeout: float = 5.0
    gps_timeout: float = 5.0
    odom_timeout: float = 2.0


class MAVROSInterface(Node):
    """
    ROS 2 Node that provides high-level interface to MAVROS.
    Handles vehicle state, setpoints, and safety checks.
    """
    
    def __init__(self, config: Optional[HardwareConfig] = None):
        super().__init__("mavros_interface")
        
        self.config = config or HardwareConfig()
        self.ns = self.config.mavros_ns
        
        # Vehicle state
        self.state = DroneState()
        self.mavros_state = State()
        self.extended_state = ExtendedState()
        self.battery = BatteryState()
        self.gps_fix = NavSatFix()
        self.home_position: Optional[HomePosition] = None
        self.imu_data = Imu()
        
        # State flags
        self.is_connected = False
        self.is_armed = False
        self.is_offboard = False
        self.is_gps_ok = False
        self.has_home_position = False
        
        # Setpoint tracking
        self.current_setpoint = VelocityCommand()
        self.last_setpoint_time = 0.0
        self.setpoint_count = 0
        
        # Publishers
        self._setup_publishers()
        
        # Subscribers
        self._setup_subscribers()
        
        # Service clients
        self._setup_services()
        
        # Timers
        self._setup_timers()
        
        self.get_logger().info("MAVROS Interface initialized")
        self.get_logger().info(f"MAVROS namespace: {self.ns}")
    
    def _setup_publishers(self):
        """Setup ROS publishers."""
        qos = QoSProfile(depth=10)
        
        # Velocity setpoint (body frame)
        self.pub_velocity = self.create_publisher(
            TwistStamped,
            f"{self.ns}/setpoint_velocity/cmd_vel",
            qos
        )
        
        # Position setpoint
        self.pub_position = self.create_publisher(
            PoseStamped,
            f"{self.ns}/setpoint_position/local",
            qos
        )
    
    def _setup_subscribers(self):
        """Setup ROS subscribers."""
        qos_sensor = qos_profile_sensor_data
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Vehicle state
        self.sub_state = self.create_subscription(
            State, f"{self.ns}/state", self._state_cb, qos_reliable
        )
        
        # Extended state
        self.sub_extended_state = self.create_subscription(
            ExtendedState, f"{self.ns}/extended_state", self._extended_state_cb, qos_sensor
        )
        
        # Battery
        self.sub_battery = self.create_subscription(
            BatteryState, f"{self.ns}/battery", self._battery_cb, qos_sensor
        )
        
        # GPS
        self.sub_gps = self.create_subscription(
            NavSatFix, f"{self.ns}/global_position/global", self._gps_cb, qos_sensor
        )
        
        # Local position (odometry)
        self.sub_local_pos = self.create_subscription(
            Odometry, f"{self.ns}/local_position/odom", self._local_pos_cb, qos_sensor
        )
        
        # Home position
        self.sub_home = self.create_subscription(
            HomePosition, f"{self.ns}/home_position/home", self._home_cb, qos_reliable
        )
        
        # IMU
        self.sub_imu = self.create_subscription(
            Imu, f"{self.ns}/imu", self._imu_cb, qos_sensor
        )
    
    def _setup_services(self):
        """Setup ROS service clients."""
        self.cli_arm = self.create_client(CommandBool, f"{self.ns}/cmd/arming")
        self.cli_mode = self.create_client(SetMode, f"{self.ns}/set_mode")
        self.cli_takeoff = self.create_client(CommandTOL, f"{self.ns}/cmd/takeoff")
        self.cli_land = self.create_client(CommandTOL, f"{self.ns}/cmd/land")
        self.cli_set_home = self.create_client(CommandHome, f"{self.ns}/cmd/set_home")
    
    def _setup_timers(self):
        """Setup ROS timers."""
        # Setpoint publishing timer
        self.timer_setpoint = self.create_timer(
            1.0 / self.config.offboard_setpoint_rate,
            self._publish_setpoint
        )
        
        # Safety check timer
        self.timer_safety = self.create_timer(
            0.5,
            self._safety_check
        )
    
    # === Callbacks ===
    
    def _state_cb(self, msg: State):
        """Handle MAVROS state messages."""
        self.mavros_state = msg
        self.is_connected = msg.connected
        self.is_armed = msg.armed
        self.is_offboard = (msg.mode == FlightMode.OFFBOARD.value)
        
        if not self.is_connected:
            self.get_logger().warn("MAVROS disconnected!")
    
    def _extended_state_cb(self, msg: ExtendedState):
        """Handle extended state messages."""
        self.extended_state = msg
        
        # Check for critical states
        if msg.land_state == ExtendedState.LAND_STATE_LANDING:
            self.get_logger().info("Vehicle is landing")
        elif msg.land_state == ExtendedState.LAND_STATE_ON_GROUND:
            self.get_logger().debug("Vehicle on ground")
    
    def _battery_cb(self, msg: BatteryState):
        """Handle battery state messages."""
        self.battery = msg
        
        # Warn on low battery
        if msg.percentage < 25.0:
            self.get_logger().warn(f"Low battery: {msg.percentage:.1f}%")
        if msg.percentage < 15.0:
            self.get_logger().error(f"Critical battery: {msg.percentage:.1f}%!")
    
    def _gps_cb(self, msg: NavSatFix):
        """Handle GPS fix messages."""
        self.gps_fix = msg
        self.is_gps_ok = (msg.status.status >= self.config.min_gps_fix_type)
    
    def _local_pos_cb(self, msg: Odometry):
        """Handle local position messages."""
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y
        self.state.z = msg.pose.pose.position.z
        self.state.vx = msg.twist.twist.linear.x
        self.state.vy = msg.twist.twist.linear.y
        self.state.vz = msg.twist.twist.linear.z
        
        q = msg.pose.pose.orientation
        self.state.quat = [q.x, q.y, q.z, q.w]
        self.state.yaw = quat_to_yaw(self.state.quat)
    
    def _home_cb(self, msg: HomePosition):
        """Handle home position messages."""
        self.home_position = msg
        self.has_home_position = True
        self.get_logger().info("Home position received")
    
    def _imu_cb(self, msg: Imu):
        """Handle IMU messages."""
        self.imu_data = msg
    
    def _publish_setpoint(self):
        """Publish velocity setpoint at fixed rate."""
        if not self.is_connected:
            return
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config.base_frame
        
        msg.twist.linear.x = self.current_setpoint.vx
        msg.twist.linear.y = self.current_setpoint.vy
        msg.twist.linear.z = self.current_setpoint.vz
        msg.twist.angular.z = self.current_setpoint.yaw_rate
        
        self.pub_velocity.publish(msg)
        self.last_setpoint_time = time.time()
        self.setpoint_count += 1
    
    def _safety_check(self):
        """Perform periodic safety checks."""
        now = time.time()
        
        # Check setpoint timeout
        if self.is_offboard and (now - self.last_setpoint_time) > self.config.offboard_timeout:
            self.get_logger().warn("Setpoint timeout in OFFBOARD mode!")
        
        # Check battery
        if self.battery.percentage < 15.0:
            self.get_logger().error("Critical battery! Consider emergency landing.")
    
    # === Control Methods ===
    
    def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Set velocity setpoint (body frame)."""
        self.current_setpoint = VelocityCommand(vx, vy, vz, yaw_rate)
    
    def set_velocity_world(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Set velocity setpoint (world frame, converted to body)."""
        from optimized.mission_common import world_to_body
        vx_body, vy_body = world_to_body(vx, vy, self.state.yaw)
        self.set_velocity(vx_body, vy_body, vz, yaw_rate)
    
    def set_position(self, x: float, y: float, z: float, yaw: Optional[float] = None):
        """Set position setpoint."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config.odom_frame
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        if yaw is not None:
            # Convert yaw to quaternion
            import math
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
        
        self.pub_position.publish(msg)
    
    async def arm(self, force: bool = False) -> bool:
        """Arm the vehicle."""
        if not self.is_connected:
            self.get_logger().error("Cannot arm: not connected")
            return False
        
        if self.is_armed:
            self.get_logger().info("Already armed")
            return True
        
        # Wait for service
        while not self.cli_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for arming service...")
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.cli_arm.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info("Vehicle armed successfully")
            return True
        else:
            self.get_logger().error("Failed to arm vehicle")
            return False
    
    async def disarm(self, force: bool = False) -> bool:
        """Disarm the vehicle."""
        if not self.is_connected:
            return False
        
        request = CommandBool.Request()
        request.value = False
        
        future = self.cli_arm.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info("Vehicle disarmed")
            return True
        return False
    
    async def set_mode(self, mode: str) -> bool:
        """Set flight mode."""
        if not self.is_connected:
            self.get_logger().error("Cannot set mode: not connected")
            return False
        
        while not self.cli_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for set_mode service...")
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.cli_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"Mode set to {mode}")
            return True
        else:
            self.get_logger().error(f"Failed to set mode to {mode}")
            return False
    
    async def set_offboard(self) -> bool:
        """Switch to OFFBOARD mode."""
        return await self.set_mode(FlightMode.OFFBOARD.value)
    
    async def set_rtl(self) -> bool:
        """Switch to RTL mode."""
        return await self.set_mode(FlightMode.AUTO.RTL.value)
    
    async def set_land(self) -> bool:
        """Switch to LAND mode."""
        return await self.set_mode(FlightMode.AUTO.LAND.value)
    
    async def takeoff(self, altitude: Optional[float] = None) -> bool:
        """Command takeoff to specified altitude."""
        altitude = altitude or self.config.default_takeoff_altitude
        
        while not self.cli_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for takeoff service...")
        
        request = CommandTOL.Request()
        request.altitude = altitude
        request.latitude = 0.0  # Use current position
        request.longitude = 0.0
        
        future = self.cli_takeoff.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info(f"Takeoff commanded to {altitude}m")
            return True
        return False
    
    def emergency_stop(self):
        """Emergency stop - switch to LAND mode immediately."""
        self.get_logger().error("EMERGENCY STOP!")
        self.current_setpoint = VelocityCommand()  # Zero velocity
        
        # Try to switch to LAND mode
        if self.is_connected:
            import threading
            threading.Thread(target=lambda: self.set_land(), daemon=True).start()
    
    def rtl(self):
        """Return to launch."""
        self.get_logger().info("Return to launch initiated")
        import threading
        threading.Thread(target=lambda: self.set_rtl(), daemon=True).start()
    
    # === State Query Methods ===
    
    def get_battery_percent(self) -> float:
        """Get battery percentage."""
        return self.battery.percentage
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get current position."""
        return (self.state.x, self.state.y, self.state.z)
    
    def get_velocity(self) -> Tuple[float, float, float]:
        """Get current velocity."""
        return (self.state.vx, self.state.vy, self.state.vz)
    
    def get_yaw(self) -> float:
        """Get current yaw angle."""
        return self.state.yaw
    
    def is_ready_for_flight(self) -> Tuple[bool, str]:
        """Check if vehicle is ready for autonomous flight."""
        if not self.is_connected:
            return False, "Not connected to MAVROS"
        
        if not self.is_gps_ok:
            return False, "GPS not ready"
        
        if not self.has_home_position:
            return False, "No home position"
        
        if self.battery.percentage < 20.0:
            return False, f"Battery too low: {self.battery.percentage:.1f}%"
        
        return True, "Ready for flight"
    
    def get_system_status(self) -> dict:
        """Get comprehensive system status."""
        return {
            "connected": self.is_connected,
            "armed": self.is_armed,
            "mode": self.mavros_state.mode if self.is_connected else "N/A",
            "offboard": self.is_offboard,
            "gps_ok": self.is_gps_ok,
            "has_home": self.has_home_position,
            "battery_percent": self.battery.percentage,
            "position": self.get_position(),
            "velocity": self.get_velocity(),
            "yaw": math.degrees(self.state.yaw),
        }


def main():
    """Test MAVROS interface."""
    rclpy.init()
    
    interface = MAVROSInterface()
    
    print("MAVROS Interface Test")
    print("=" * 40)
    
    # Wait for connection
    print("Waiting for MAVROS connection...")
    timeout = 30.0
    start = time.time()
    
    while not interface.is_connected and (time.time() - start) < timeout:
        rclpy.spin_once(interface, timeout_sec=0.1)
    
    if not interface.is_connected:
        print("Failed to connect to MAVROS!")
        interface.destroy_node()
        rclpy.shutdown()
        return
    
    print("Connected!")
    
    # Print status
    status = interface.get_system_status()
    print("\nSystem Status:")
    for key, value in status.items():
        print(f"  {key}: {value}")
    
    # Check ready
    ready, msg = interface.is_ready_for_flight()
    print(f"\nReady for flight: {ready} ({msg})")
    
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
