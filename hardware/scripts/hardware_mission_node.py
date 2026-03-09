#!/usr/bin/env python3
"""
Hardware Mission Node - MAVROS Integration

This module provides the hardware deployment version of the drone mission
with MAVROS integration for Pixhawk flight controllers.

Key features:
- MAVROS state management
- Flight mode transitions
- Real sensor integration
- Hardware-specific safety checks
- GPS-based navigation fallback
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from mavros_msgs.msg import State, ExtendedState, BatteryStatus, HomePosition, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import Imu, NavSatFix, LaserScan, Image
from std_msgs.msg import Bool, Header

from cv_bridge import CvBridge
import cv2
import numpy as np


class FlightMode(Enum):
    """MAVLink flight modes"""
    MANUAL = "MANUAL"
    STABILIZE = "STABILIZE"
    ALTITUDE = "ALTCTL"
    POSITION = "POSCONTROL"
    OFFBOARD = "OFFBOARD"
    LAND = "AUTO.LAND"
    RTL = "AUTO.RTL"
    LOITER = "AUTO.LOITER"


class HardwareMissionState(Enum):
    """Mission state machine states"""
    INIT = auto()
    PREFLIGHT_CHECK = auto()
    WAITING_ARM = auto()
    TAKEOFF = auto()
    MAPPING = auto()
    RETURN_HOME = auto()
    LANDING = auto()
    LANDED = auto()
    ERROR = auto()


@dataclass
class FCUState:
    """Flight controller state container"""
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = ""
    system_status: int = 0

    # Extended state
    vtol_state: int = 0
    landed_state: int = 0  # 0=UNKNOWN, 1=ON_GROUND, 2=IN_AIR, 3=TAKEOFF, 4=LANDING

    # Battery
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_remaining: float = 100.0

    # GPS
    gps_fix: bool = False
    gps_lat: float = 0.0
    gps_lon: float = 0.0
    gps_alt: float = 0.0
    gps_satellites: int = 0

    # Home position
    home_lat: float = 0.0
    home_lon: float = 0.0
    home_alt: float = 0.0


class HardwareMissionNode(Node):
    """
    Hardware deployment mission node with MAVROS integration

    This node handles:
    - MAVROS state synchronization
    - Flight mode transitions
    - Offboard control
    - Real sensor data processing
    - Hardware-specific failsafes
    """

    def __init__(self):
        super().__init__("hardware_mission_node")

        # Declare parameters
        self._declare_parameters()

        # Initialize state
        self.fcu_state = FCUState()
        self.mission_state = HardwareMissionState.INIT
        self.bridge = CvBridge()

        # Position tracking
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0
        self.yaw = 0.0

        # Target position for offboard
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 2.0  # Default takeoff altitude

        # Safety parameters
        self.min_gps_satellites = 6
        self.min_battery_takeoff = 50.0
        self.max_distance_from_home = 100.0
        self.max_altitude = 30.0

        # Initialize interfaces
        self._init_publishers()
        self._init_subscribers()
        self._init_services()
        self._init_timers()

        # State change service clients
        self.arming_client = None
        self.set_mode_client = None
        self.takeoff_client = None
        self.land_client = None

        self.get_logger().info("Hardware mission node initialized")
        self.get_logger().info("Waiting for FCU connection...")

    def _declare_parameters(self):
        """Declare all parameters"""
        self.declare_parameter("takeoff_altitude", 2.5)
        self.declare_parameter("mapping_altitude", 4.0)
        self.declare_parameter("max_xy_speed", 1.0)
        self.declare_parameter("max_z_speed", 0.5)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("offboard_setpoint_rate", 20.0)
        self.declare_parameter("min_gps_satellites", 6)
        self.declare_parameter("min_battery_takeoff", 50.0)
        self.declare_parameter("geofence_radius", 100.0)
        self.declare_parameter("max_altitude", 30.0)

    def _init_publishers(self):
        """Initialize publishers"""
        qos = QoSProfile(depth=10)

        # Offboard control
        self.pub_position_target = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", qos
        )
        self.pub_velocity_cmd = self.create_publisher(
            Twist, "/mavros/setpoint_velocity/cmd_vel", qos
        )

        # Manual override
        self.pub_manual_mode = self.create_publisher(Bool, "/manual_mode", qos)
        self.pub_manual_cmd = self.create_publisher(Twist, "/manual_cmd_vel", qos)

    def _init_subscribers(self):
        """Initialize subscribers"""
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        callback_group = ReentrantCallbackGroup()

        # MAVROS state
        self.sub_state = self.create_subscription(
            State, "/mavros/state", self._state_callback, qos,
            callback_group=callback_group
        )
        self.sub_extended_state = self.create_subscription(
            ExtendedState, "/mavros/extended_state", self._extended_state_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_battery = self.create_subscription(
            BatteryStatus, "/mavros/battery", self._battery_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_home = self.create_subscription(
            HomePosition, "/mavros/home_position/home", self._home_callback, qos,
            callback_group=callback_group
        )

        # Position
        self.sub_local_pos = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._local_pose_callback,
            sensor_qos, callback_group=callback_group
        )
        self.sub_gps = self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._gps_callback,
            sensor_qos, callback_group=callback_group
        )

        # Sensors
        self.sub_imu = self.create_subscription(
            Imu, "/mavros/imu/data", self._imu_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_lidar = self.create_subscription(
            LaserScan, "/scan", self._lidar_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_camera_front = self.create_subscription(
            Image, "/front_camera/image_raw", self._camera_callback, sensor_qos,
            callback_group=callback_group
        )

    def _init_services(self):
        """Initialize service clients"""
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

    def _init_timers(self):
        """Initialize timers"""
        control_rate = self.get_parameter("control_rate").value
        offboard_rate = self.get_parameter("offboard_setpoint_rate").value

        self.control_timer = self.create_timer(1.0 / control_rate, self._control_tick)
        self.offboard_timer = self.create_timer(1.0 / offboard_rate, self._publish_offboard_setpoint)

    # ============== Callbacks ==============

    def _state_callback(self, msg: State):
        """Process FCU state"""
        self.fcu_state.connected = msg.connected
        self.fcu_state.armed = msg.armed
        self.fcu_state.guided = msg.guided
        self.fcu_state.mode = msg.mode
        self.fcu_state.system_status = msg.system_status

        if msg.connected and self.mission_state == HardwareMissionState.INIT:
            self.get_logger().info(f"FCU connected, mode: {msg.mode}")
            self.mission_state = HardwareMissionState.PREFLIGHT_CHECK

    def _extended_state_callback(self, msg: ExtendedState):
        """Process extended FCU state"""
        self.fcu_state.vtol_state = msg.vtol_state
        self.fcu_state.landed_state = msg.landed_state

    def _battery_callback(self, msg: BatteryStatus):
        """Process battery status"""
        self.fcu_state.battery_voltage = msg.voltage
        self.fcu_state.battery_current = msg.current
        self.fcu_state.battery_remaining = msg.remaining * 100.0

    def _home_callback(self, msg: HomePosition):
        """Process home position"""
        self.fcu_state.home_lat = msg.geo.latitude
        self.fcu_state.home_lon = msg.geo.longitude
        self.fcu_state.home_alt = msg.geo.altitude

    def _local_pose_callback(self, msg: PoseStamped):
        """Process local position"""
        self.local_x = msg.pose.position.x
        self.local_y = msg.pose.position.y
        self.local_z = msg.pose.position.z

        # Extract yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def _gps_callback(self, msg: NavSatFix):
        """Process GPS data"""
        self.fcu_state.gps_fix = msg.status.status >= 0
        self.fcu_state.gps_lat = msg.latitude
        self.fcu_state.gps_lon = msg.longitude
        self.fcu_state.gps_alt = msg.altitude

    def _imu_callback(self, msg: Imu):
        """Process IMU data"""
        pass  # Can be used for attitude estimation

    def _lidar_callback(self, msg: LaserScan):
        """Process LiDAR data for obstacle avoidance"""
        pass  # Implement obstacle avoidance

    def _camera_callback(self, msg: Image):
        """Process camera data"""
        pass  # Implement boundary detection

    # ============== Control Functions ==============

    def _publish_offboard_setpoint(self):
        """Publish offboard position setpoint"""
        if self.fcu_state.mode != FlightMode.OFFBOARD.value:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z

        # Set yaw
        msg.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.pub_position_target.publish(msg)

    async def set_offboard_mode(self) -> bool:
        """Switch to OFFBOARD mode"""
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Set mode service not available")
            return False

        request = SetMode.Request()
        request.custom_mode = FlightMode.OFFBOARD.value

        future = self.set_mode_client.call_async(request)
        await future

        if future.result() and future.result().mode_sent:
            self.get_logger().info("OFFBOARD mode set")
            return True
        return False

    async def arm(self) -> bool:
        """Arm the vehicle"""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return False

        request = CommandBool.Request()
        request.value = True

        future = self.arming_client.call_async(request)
        await future

        if future.result() and future.result().success:
            self.get_logger().info("Vehicle armed")
            return True
        return False

    async def disarm(self) -> bool:
        """Disarm the vehicle"""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            return False

        request = CommandBool.Request()
        request.value = False

        future = self.arming_client.call_async(request)
        await future

        return future.result() and future.result().success

    def send_velocity_command(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Send velocity command in body frame"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.pub_velocity_cmd.publish(msg)

    def _preflight_check(self) -> Tuple[bool, str]:
        """Perform preflight checks"""
        checks = []

        # GPS check
        if not self.fcu_state.gps_fix:
            checks.append("GPS: No fix")
        elif self.fcu_state.gps_satellites < self.min_gps_satellites:
            checks.append(f"GPS: Only {self.fcu_state.gps_satellites} satellites")

        # Battery check
        if self.fcu_state.battery_remaining < self.min_battery_takeoff:
            checks.append(f"Battery: {self.fcu_state.battery_remaining:.1f}% (need {self.min_battery_takeoff}%)")

        # Connection check
        if not self.fcu_state.connected:
            checks.append("FCU: Not connected")

        # Home position check
        if self.fcu_state.home_lat == 0.0 and self.fcu_state.home_lon == 0.0:
            checks.append("Home: Not set")

        if checks:
            return False, "; ".join(checks)
        return True, "All checks passed"

    def _control_tick(self):
        """Main control loop"""
        now = self.get_clock().now()

        # State machine
        if self.mission_state == HardwareMissionState.PREFLIGHT_CHECK:
            ok, msg = self._preflight_check()
            if ok:
                self.get_logger().info("Preflight check passed")
                self.mission_state = HardwareMissionState.WAITING_ARM
            else:
                self.get_logger().warn(f"Preflight check: {msg}")

        elif self.mission_state == HardwareMissionState.WAITING_ARM:
            # Wait for arm command or auto-arm
            pass

        elif self.mission_state == HardwareMissionState.TAKEOFF:
            # Monitor takeoff
            if abs(self.local_z - self.target_z) < 0.3:
                self.get_logger().info(f"Reached takeoff altitude: {self.local_z:.1f}m")
                self.mission_state = HardwareMissionState.MAPPING

        elif self.mission_state == HardwareMissionState.MAPPING:
            # Execute mapping mission
            self._execute_mapping()

        elif self.missionState == HardwareMissionState.RETURN_HOME:
            # RTL
            self.target_x = 0.0
            self.target_y = 0.0

        elif self.mission_state == HardwareMissionState.LANDING:
            # Monitor landing
            if self.fcu_state.landed_state == 1:  # ON_GROUND
                self.get_logger().info("Landed")
                self.mission_state = HardwareMissionState.LANDED

        # Safety checks
        self._safety_checks()

    def _execute_mapping(self):
        """Execute mapping mission logic"""
        # Implement waypoint navigation with obstacle avoidance
        pass

    def _safety_checks(self):
        """Run safety checks"""
        # Geofence
        distance = math.hypot(self.local_x, self.local_y)
        if distance > self.max_distance_from_home:
            self.get_logger().error(f"GEOFENCE: {distance:.1f}m from home")
            # Trigger RTL

        # Altitude limit
        if self.local_z > self.max_altitude:
            self.get_logger().error(f"ALTITUDE LIMIT: {self.local_z:.1f}m")

        # Battery critical
        if self.fcu_state.battery_remaining < 20.0:
            self.get_logger().error(f"LOW BATTERY: {self.fcu_state.battery_remaining:.1f}%")
            # Trigger emergency landing


def main(args=None):
    rclpy.init(args=args)
    node = HardwareMissionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
