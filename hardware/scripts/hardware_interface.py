#!/usr/bin/env python3
"""
Hardware Mission Node for MAVROS Integration
=============================================
This module provides the bridge between the mission logic and real hardware
via MAVROS. It handles:
- MAVROS topic subscriptions and publishers
- Flight mode management
- Arming/disarming
- Position and velocity control
- Safety monitoring

Compatible with:
- Pixhawk 4/6 flight controllers
- PX4 and ArduPilot firmware
- ROS 2 Jazzy
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, BatteryState, Imu
from std_msgs.msg import Bool, Float64

# MAVROS messages
try:
    from mavros_msgs.msg import State, PositionTarget, ExtendedState, HomePosition
    from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False
    print("[WARN] MAVROS messages not available. Hardware mode disabled.")

from cv_bridge import CvBridge
import numpy as np


class FlightMode(Enum):
    """MAVROS flight modes."""
    MANUAL = "MANUAL"
    STABILIZE = "STABILIZE"
    ALTCTL = "ALTCTL"  # Altitude control
    POSCTL = "POSCTL"  # Position control
    OFFBOARD = "OFFBOARD"
    AUTO_TAKEOFF = "AUTO.TAKEOFF"
    AUTO_LAND = "AUTO.LAND"
    AUTO_RTL = "AUTO.RTL"
    AUTO_MISSION = "AUTO.MISSION"


class HardwareState(Enum):
    """Hardware interface states."""
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ARMED = auto()
    IN_FLIGHT = auto()
    ERROR = auto()


@dataclass
class VehicleState:
    """Current vehicle state from MAVROS."""
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = ""
    
    # Position (local NED frame)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    # Velocity (local NED frame)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    
    # Attitude
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    # Battery
    battery_percent: float = 100.0
    battery_voltage: float = 16.8
    
    # GPS
    gps_fix: int = 0
    num_satellites: int = 0
    
    # Safety
    landed: bool = True
    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = 0.0


class HardwareInterface(Node):
    """
    Hardware interface node for MAVROS communication.
    
    Provides a unified interface for:
    - Reading sensor data (compatible with simulation topics)
    - Sending velocity commands
    - Managing flight modes and arming
    - Safety monitoring
    """
    
    def __init__(self, config_path: Optional[str] = None):
        super().__init__("hardware_interface")
        
        if not MAVROS_AVAILABLE:
            self.get_logger().error("MAVROS not available. Cannot initialize hardware interface.")
            raise RuntimeError("MAVROS messages not installed")
        
        # State
        self.state = VehicleState()
        self.bridge = CvBridge()
        self.callback_group = ReentrantCallbackGroup()
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Setup interfaces
        self._setup_mavros_subscribers()
        self._setup_mavros_publishers()
        self._setup_mavros_services()
        self._setup_sensor_publishers()
        self._setup_timers()
        
        # Target position for offboard control
        self.target_position = PositionTarget()
        self.target_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.target_position.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        
        self.get_logger().info("Hardware interface initialized. Waiting for FCU connection...")
    
    def _setup_mavros_subscribers(self):
        """Setup MAVROS topic subscribers."""
        # State
        self.sub_state = self.create_subscription(
            State, "/mavros/state", self._state_callback, self.mavros_qos
        )
        
        # Extended state (for landed state)
        self.sub_extended_state = self.create_subscription(
            ExtendedState, "/mavros/extended_state", self._extended_state_callback, self.mavros_qos
        )
        
        # Position (local NED)
        self.sub_local_pos = self.create_subscription(
            Odometry, "/mavros/local_position/odom", self._local_position_callback, self.mavros_qos
        )
        
        # Battery
        self.sub_battery = self.create_subscription(
            BatteryState, "/mavros/battery", self._battery_callback, self.mavros_qos
        )
        
        # Home position
        self.sub_home = self.create_subscription(
            HomePosition, "/mavros/home_position/home", self._home_callback, self.mavros_qos
        )
        
        # IMU
        self.sub_imu = self.create_subscription(
            Imu, "/mavros/imu/data", self._imu_callback, self.mavros_qos
        )
    
    def _setup_mavros_publishers(self):
        """Setup MAVROS topic publishers."""
        # Velocity setpoint
        self.pub_velocity = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10
        )
        
        # Position setpoint
        self.pub_position = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )
    
    def _setup_mavros_services(self):
        """Setup MAVROS service clients."""
        # Arming
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        
        # Mode setting
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        
        # Takeoff
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        
        # Landing
        self.cli_land = self.create_client(CommandTOL, "/mavros/cmd/land")
    
    def _setup_sensor_publishers(self):
        """Setup publishers to republish data in simulation-compatible format."""
        # Republish odometry in simulation format
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        
        # Enable signal
        self.pub_enable = self.create_publisher(Bool, "/drone_enable", 10)
        
        # Velocity command input
        self.sub_cmd_vel = self.create_subscription(
            geometry_msgs.msg.Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )
    
    def _setup_timers(self):
        """Setup control timers."""
        # Connection watchdog
        self.timer_watchdog = self.create_timer(1.0, self._watchdog_callback)
        
        # Offboard setpoint stream (required for offboard mode)
        self.timer_setpoint = self.create_timer(0.05, self._setpoint_callback)  # 20 Hz
    
    # ==================== Callbacks ====================
    
    def _state_callback(self, msg: State):
        """Handle MAVROS state updates."""
        prev_connected = self.state.connected
        prev_armed = self.state.armed
        
        self.state.connected = msg.connected
        self.state.armed = msg.armed
        self.state.guided = msg.guided
        self.state.mode = msg.mode
        
        if msg.connected and not prev_connected:
            self.get_logger().info("FCU connected!")
        elif not msg.connected and prev_connected:
            self.get_logger().warn("FCU disconnected!")
        
        if msg.armed and not prev_armed:
            self.get_logger().info("Vehicle ARMED")
        elif not msg.armed and prev_armed:
            self.get_logger().info("Vehicle DISARMED")
    
    def _extended_state_callback(self, msg: ExtendedState):
        """Handle extended state for landed state."""
        # landed_state: 1 = ON_GROUND, 2 = IN_AIR, 3 = TAKEOFF, 4 = LANDING
        self.state.landed = (msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)
    
    def _local_position_callback(self, msg: Odometry):
        """Handle local position updates."""
        # NED frame: x=North, y=East, z=Down
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y
        self.state.z = -msg.pose.pose.position.z  # Convert to altitude
        
        self.state.vx = msg.twist.twist.linear.x
        self.state.vy = msg.twist.twist.linear.y
        self.state.vz = -msg.twist.twist.linear.z
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.state.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        # Republish in simulation format (ENU frame)
        self._publish_sim_odom(msg)
    
    def _publish_sim_odom(self, msg: Odometry):
        """Republish odometry in simulation-compatible ENU format."""
        sim_msg = Odometry()
        sim_msg.header = msg.header
        sim_msg.header.frame_id = "odom"
        sim_msg.child_frame_id = "base_link"
        
        # Convert NED to ENU (x=East, y=North, z=Up)
        sim_msg.pose.pose.position.x = msg.pose.pose.position.y  # East
        sim_msg.pose.pose.position.y = msg.pose.pose.position.x  # North
        sim_msg.pose.pose.position.z = -msg.pose.pose.position.z  # Up
        
        # Rotate orientation from NED to ENU
        q = msg.pose.pose.orientation
        # ENU quaternion from NED: rotate 90deg about z, then 180deg about x
        # Simplified: swap and negate appropriately
        sim_msg.pose.pose.orientation.x = q.y
        sim_msg.pose.pose.orientation.y = q.x
        sim_msg.pose.pose.orientation.z = -q.z
        sim_msg.pose.pose.orientation.w = q.w
        
        # Convert velocity
        sim_msg.twist.twist.linear.x = msg.twist.twist.linear.y
        sim_msg.twist.twist.linear.y = msg.twist.twist.linear.x
        sim_msg.twist.twist.linear.z = -msg.twist.twist.linear.z
        
        self.pub_odom.publish(sim_msg)
    
    def _battery_callback(self, msg: BatteryState):
        """Handle battery state updates."""
        self.state.battery_percent = msg.percentage
        self.state.battery_voltage = msg.voltage
        
        if msg.percentage < 20.0:
            self.get_logger().warn(f"Low battery: {msg.percentage:.1f}%")
    
    def _home_callback(self, msg: HomePosition):
        """Handle home position updates."""
        self.state.home_x = msg.position.x
        self.state.home_y = msg.position.y
        self.state.home_z = msg.position.z
        self.get_logger().info(f"Home position set: ({self.state.home_x:.2f}, {self.state.home_y:.2f})")
    
    def _imu_callback(self, msg: Imu):
        """Handle IMU updates for attitude."""
        # Extract roll/pitch from quaternion
        q = msg.orientation
        self.state.roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )
        self.state.pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
    
    def _cmd_vel_callback(self, msg: geometry_msgs.msg.Twist):
        """Handle velocity commands from mission controller."""
        # Convert ENU velocity to NED
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = self.get_clock().now().to_msg()
        velocity_msg.header.frame_id = "base_link"
        
        # ENU to NED: x=N, y=E -> x=E, y=N, z stays, yaw rotates
        velocity_msg.twist.linear.x = msg.linear.y   # North
        velocity_msg.twist.linear.y = msg.linear.x   # East
        velocity_msg.twist.linear.z = -msg.linear.z  # Up -> Down
        velocity_msg.twist.angular.z = msg.angular.z
        
        self.pub_velocity.publish(velocity_msg)
    
    def _watchdog_callback(self):
        """Check connection and safety status."""
        if not self.state.connected:
            self.get_logger().warn_throttle(5.0, "Waiting for FCU connection...")
            return
        
        # Check battery
        if self.state.battery_percent < 15.0:
            self.get_logger().error(f"CRITICAL: Battery at {self.state.battery_percent:.1f}%")
    
    def _setpoint_callback(self):
        """Stream offboard setpoints (required to maintain offboard mode)."""
        if not self.state.connected:
            return
        
        # Publish a minimal velocity setpoint to maintain offboard mode
        if self.state.mode == FlightMode.OFFBOARD.value:
            # Just republish current target
            pass  # Velocity commands are published separately
    
    # ==================== Control Methods ====================
    
    def arm(self, force: bool = False) -> bool:
        """Arm the vehicle."""
        if not self.state.connected:
            self.get_logger().error("Cannot arm: FCU not connected")
            return False
        
        if self.state.armed:
            self.get_logger().info("Already armed")
            return True
        
        if not self.cli_arm.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return False
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.cli_arm.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("Arm command sent successfully")
            return True
        else:
            self.get_logger().error(f"Arm command failed: {future.result()}")
            return False
    
    def disarm(self, force: bool = False) -> bool:
        """Disarm the vehicle."""
        if not self.state.connected:
            return False
        
        if not self.state.armed:
            return True
        
        if not self.cli_arm.wait_for_service(timeout_sec=2.0):
            return False
        
        request = CommandBool.Request()
        request.value = False
        
        future = self.cli_arm.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        return future.result() and future.result().success
    
    def set_mode(self, mode: str) -> bool:
        """Set flight mode."""
        if not self.state.connected:
            self.get_logger().error("Cannot set mode: FCU not connected")
            return False
        
        if not self.cli_mode.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Mode service not available")
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.cli_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"Mode change to {mode} sent")
            return True
        else:
            self.get_logger().error(f"Mode change to {mode} failed")
            return False
    
    def takeoff(self, altitude: float, timeout: float = 30.0) -> bool:
        """Command takeoff to specified altitude."""
        if not self.state.connected or not self.state.armed:
            return False
        
        if not self.cli_takeoff.wait_for_service(timeout_sec=2.0):
            # Try using position control instead
            self.get_logger().info("Takeoff service not available, using offboard climb")
            return self._offboard_takeoff(altitude)
        
        request = CommandTOL.Request()
        request.altitude = altitude
        request.latitude = 0  # Current position
        request.longitude = 0
        request.min_pitch = 0
        
        future = self.cli_takeoff.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        return future.result() and future.result().success
    
    def _offboard_takeoff(self, altitude: float) -> bool:
        """Perform offboard mode takeoff."""
        # Switch to offboard mode first
        if self.state.mode != FlightMode.OFFBOARD.value:
            if not self.set_mode(FlightMode.OFFBOARD.value):
                return False
            time.sleep(0.5)
        
        # Publish climb setpoint
        self.target_position.position.z = -altitude  # NED frame
        self.target_position.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        
        start_time = time.time()
        while time.time() - start_time < 30.0:
            self.pub_position.publish(self.target_position)
            if abs(self.state.z - altitude) < 0.5:
                return True
            time.sleep(0.05)
        
        return False
    
    def land(self, timeout: float = 60.0) -> bool:
        """Command landing."""
        if not self.state.connected or not self.state.armed:
            return False
        
        # Try auto land first
        if self.cli_land.wait_for_service(timeout_sec=2.0):
            request = CommandTOL.Request()
            request.altitude = 0
            
            future = self.cli_land.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() and future.result().success:
                # Wait for landing
                start_time = time.time()
                while time.time() - start_time < timeout:
                    if self.state.landed:
                        self.get_logger().info("Landing complete")
                        return True
                    time.sleep(0.1)
        
        # Fallback to RTL
        self.get_logger().warn("Auto land failed, trying RTL")
        return self.set_mode(FlightMode.AUTO_RTL.value)
    
    def return_to_launch(self) -> bool:
        """Command return to launch."""
        return self.set_mode(FlightMode.AUTO_RTL.value)
    
    def publish_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Publish velocity setpoint (NED frame expected by MAVROS)."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate
        
        self.pub_velocity.publish(msg)
    
    def is_ready_for_flight(self) -> Tuple[bool, str]:
        """Check if vehicle is ready for autonomous flight."""
        if not self.state.connected:
            return False, "FCU not connected"
        
        if self.state.battery_percent < 30.0:
            return False, f"Low battery: {self.state.battery_percent:.1f}%"
        
        if not self.state.armed:
            return False, "Not armed"
        
        if self.state.mode not in [FlightMode.OFFBOARD.value, FlightMode.POSCTL.value]:
            return False, f"Wrong mode: {self.state.mode}"
        
        return True, "Ready"
    
    # ==================== Simulation Compatibility ====================
    
    def publish_enable(self, enabled: bool):
        """Publish enable signal (simulation compatibility)."""
        msg = Bool()
        msg.data = enabled
        self.pub_enable.publish(msg)
        
        # For hardware, this corresponds to switching to offboard mode
        if enabled and self.state.connected and self.state.armed:
            if self.state.mode != FlightMode.OFFBOARD.value:
                self.set_mode(FlightMode.OFFBOARD.value)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HardwareInterface()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Failed to initialize hardware interface: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
