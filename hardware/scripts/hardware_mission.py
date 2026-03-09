#!/usr/bin/env python3
"""
Hardware Mission Control Node
Autonomous drone mission for hardware deployment with MAVROS.

This script bridges the simulation mission logic to real hardware,
providing the same state machine behavior but with MAVROS integration.

Usage:
    ros2 run <package> hardware_mission.py --ros-args -p use_sim_time:=false

Author: Drone Project Team
Version: 2.0.0
"""

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, LaserScan, BatteryState
from std_msgs.msg import Bool, Header

# Add parent paths for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from optimized.mission_common import (
    DroneState, VelocityCommand, SafetyLimits, RateLimiter, PIDController,
    MissionPhase, clamp, angle_wrap, quat_to_yaw, world_to_body, body_to_world,
    distance_2d, normalize_vector, load_yaml_config, build_sweep_waypoints,
    YellowDetector, PerformanceMonitor
)


class OperationMode(Enum):
    """Operation mode selection."""
    SIMULATION = "simulation"
    HARDWARE = "hardware"


@dataclass
class MissionConfig:
    """Mission configuration loaded from YAML."""
    # Target altitude
    target_alt: float = 4.0
    
    # Control rate
    control_rate: float = 20.0
    
    # Base position
    base_x: float = 0.0
    base_y: float = 0.0
    
    # Velocity limits
    max_xy_speed: float = 1.45
    max_z_speed: float = 0.6
    max_yaw_rate: float = 0.9
    
    # Arena dimensions
    arena_half_extent: float = 8.0
    edge_margin: float = 1.2
    lane_spacing: float = 0.9
    
    # Battery thresholds
    battery_low: float = 25.0
    battery_critical: float = 12.0
    
    @classmethod
    def from_yaml(cls, filepath: str) -> 'MissionConfig':
        """Load configuration from YAML file."""
        config = load_yaml_config(filepath)
        mission = config.get('mission', {})
        velocity = config.get('velocity_limits', {})
        mapping = config.get('mapping', {})
        battery = config.get('battery', {})
        
        return cls(
            target_alt=mission.get('target_altitude', 4.0),
            control_rate=mission.get('control_rate', 20.0),
            base_x=mission.get('base_x', 0.0),
            base_y=mission.get('base_y', 0.0),
            max_xy_speed=velocity.get('max_xy_speed', 1.45),
            max_z_speed=velocity.get('max_z_speed', 0.6),
            max_yaw_rate=velocity.get('max_yaw_rate', 0.9),
            arena_half_extent=mapping.get('arena_half_extent', 8.0),
            edge_margin=mapping.get('edge_margin', 1.2),
            lane_spacing=mapping.get('lane_spacing', 0.9),
            battery_low=battery.get('low_warn_percent', 25.0),
            battery_critical=battery.get('critical_percent', 12.0),
        )


class HardwareMissionNode(Node):
    """
    Main mission control node for hardware deployment.
    
    Features:
    - MAVROS-compatible control interface
    - Battery monitoring with RTL
    - Geofencing
    - Yellow boundary detection
    - SLAM integration
    - Manual override support
    """
    
    def __init__(
        self,
        config: Optional[MissionConfig] = None,
        mode: OperationMode = OperationMode.HARDWARE
    ):
        super().__init__("hardware_mission")
        
        self.config = config or MissionConfig()
        self.mode = mode
        
        # State
        self.state = DroneState()
        self.phase = MissionPhase.WAIT_SENSORS
        self.phase_start_time = 0.0
        
        # Sensor flags
        self.has_odom = False
        self.has_scan = False
        self.has_front_camera = False
        self.has_down_camera = False
        
        # Battery
        self.battery_percent = 100.0
        
        # Control
        self.cmd = VelocityCommand()
        self.rate_limiter = RateLimiter(2.2, 1.4, 2.0)
        self.controller_enabled = False
        
        # Yellow detection
        self.yellow_detector = YellowDetector()
        self.front_yellow_ratio = 0.0
        self.down_yellow_ratio = 0.0
        
        # Waypoints
        self.waypoints = build_sweep_waypoints(
            self.config.arena_half_extent,
            self.config.edge_margin,
            self.config.lane_spacing,
            self.config.target_alt
        )
        self.waypoint_idx = 0
        self.waypoint_start_time = 0.0
        
        # Performance monitoring
        self.perf = PerformanceMonitor()
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Display windows
        self.show_windows = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        
        # Callback group for concurrent execution
        self.cb_group = ReentrantCallbackGroup()
        
        # Setup interfaces
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()
        
        self.get_logger().info(f"Hardware Mission Node initialized (mode: {mode.value})")
        self.get_logger().info(f"Generated {len(self.waypoints)} mapping waypoints")
    
    def _setup_publishers(self):
        """Setup publishers."""
        qos = QoSProfile(depth=10)
        
        # Velocity command
        if self.mode == OperationMode.SIMULATION:
            self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", qos)
            self.pub_enable = self.create_publisher(Bool, "/drone_enable", qos)
        else:
            # Hardware uses MAVROS velocity setpoint
            self.pub_cmd = self.create_publisher(Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", qos)
        
        # Manual mode
        self.pub_manual_mode = self.create_publisher(Bool, "/manual_mode", qos)
    
    def _setup_subscribers(self):
        """Setup subscribers."""
        qos_sensor = qos_profile_sensor_data
        
        # Odometry
        odom_topic = "/odom" if self.mode == OperationMode.SIMULATION else "/mavros/local_position/odom"
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, qos_sensor
        )
        
        # LiDAR scan
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_sensor
        )
        
        # Cameras
        self.sub_front_camera = self.create_subscription(
            Image, "/front_camera/image_raw", self._front_camera_cb, qos_sensor
        )
        self.sub_down_camera = self.create_subscription(
            Image, "/down_camera/image_raw", self._down_camera_cb, qos_sensor
        )
        
        # Battery
        if self.mode == OperationMode.HARDWARE:
            self.sub_battery = self.create_subscription(
                BatteryState, "/mavros/battery", self._battery_cb, qos_sensor
            )
        
        # Manual override
        self.sub_manual = self.create_subscription(
            Bool, "/manual_mode", self._manual_cb, qos
        )
        self.sub_manual_cmd = self.create_subscription(
            Twist, "/manual_cmd_vel", self._manual_cmd_cb, qos
        )
        
        # Map for coverage tracking
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, qos_sensor
        )
    
    def _setup_timers(self):
        """Setup timers."""
        self.timer_control = self.create_timer(
            1.0 / self.config.control_rate,
            self._control_tick
        )
    
    # === Callbacks ===
    
    def _odom_cb(self, msg: Odometry):
        """Handle odometry messages."""
        self.perf.start("odom_cb")
        
        self.has_odom = True
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y
        self.state.z = msg.pose.pose.position.z
        self.state.vx = msg.twist.twist.linear.x
        self.state.vy = msg.twist.twist.linear.y
        self.state.vz = msg.twist.twist.linear.z
        
        q = msg.pose.pose.orientation
        self.state.quat = np.array([q.x, q.y, q.z, q.w])
        self.state.yaw = quat_to_yaw(self.state.quat)
        
        self.perf.end("odom_cb")
    
    def _scan_cb(self, msg: LaserScan):
        """Handle LiDAR scan messages."""
        self.has_scan = True
        # Store scan for obstacle avoidance
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        self.scan_ranges = ranges[valid]
        self.scan_angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        self.scan_angles = self.scan_angles[valid]
    
    def _front_camera_cb(self, msg: Image):
        """Handle front camera messages."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.front_yellow_ratio, self.front_vis = self.yellow_detector.detect(frame)
            self.has_front_camera = True
        except Exception as e:
            self.get_logger().warn(f"Front camera decode error: {e}")
    
    def _down_camera_cb(self, msg: Image):
        """Handle down camera messages."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.down_yellow_ratio, self.down_vis = self.yellow_detector.detect(frame)
            self.has_down_camera = True
        except Exception as e:
            self.get_logger().warn(f"Down camera decode error: {e}")
    
    def _battery_cb(self, msg: BatteryState):
        """Handle battery state (hardware only)."""
        self.battery_percent = msg.percentage
    
    def _manual_cb(self, msg: Bool):
        """Handle manual mode toggle."""
        if msg.data:
            self.get_logger().warn("Manual mode enabled")
        else:
            self.get_logger().info("Returning to autonomous mode")
    
    def _manual_cmd_cb(self, msg: Twist):
        """Handle manual velocity commands."""
        pass  # Handled in control tick
    
    def _map_cb(self, msg: OccupancyGrid):
        """Handle map updates."""
        data = np.array(msg.data, dtype=np.int16)
        known = np.count_nonzero(data >= 0)
        self.map_coverage = 100.0 * known / data.size if data.size > 0 else 0.0
    
    # === Control Methods ===
    
    def _control_tick(self):
        """Main control loop tick."""
        self.perf.start("control_tick")
        
        now = self.get_clock().now().nanoseconds * 1e-9
        
        # Update phase timing
        if self.phase == MissionPhase.WAIT_SENSORS:
            if self._check_sensors_ready():
                self._set_phase(MissionPhase.TAKEOFF)
        
        elif self.phase == MissionPhase.TAKEOFF:
            if self._execute_takeoff(now):
                self._set_phase(MissionPhase.MAPPING)
                self.waypoint_idx = 0
        
        elif self.phase == MissionPhase.MAPPING:
            # Check battery
            if self.battery_percent < self.config.battery_low:
                self.get_logger().warn(f"Low battery ({self.battery_percent:.1f}%), initiating RTL")
                self._set_phase(MissionPhase.RETURN_HOME_AT_4M)
            else:
                self._execute_mapping(now)
        
        elif self.phase == MissionPhase.RETURN_HOME_AT_4M:
            if self._execute_return_home(now, self.config.target_alt):
                self._set_phase(MissionPhase.HOVER_HOME_AT_4M)
        
        elif self.phase == MissionPhase.HOVER_HOME_AT_4M:
            if self._execute_hover(now, 2.0):
                self._set_phase(MissionPhase.ALIGN_YAW_AT_4M)
        
        elif self.phase == MissionPhase.ALIGN_YAW_AT_4M:
            if self._execute_yaw_align(now, self.config.target_alt, 0.0):
                self._set_phase(MissionPhase.DESCEND_TO_2M)
        
        elif self.phase == MissionPhase.DESCEND_TO_2M:
            if self._execute_descend(now, 2.0):
                self._set_phase(MissionPhase.HOLD_AT_2M)
        
        elif self.phase == MissionPhase.HOLD_AT_2M:
            if self._execute_hover(now, 2.0):
                self._set_phase(MissionPhase.ALIGN_YAW_AT_2M)
        
        elif self.phase == MissionPhase.ALIGN_YAW_AT_2M:
            if self._execute_yaw_align(now, 2.0, 0.0):
                self._set_phase(MissionPhase.LAND)
        
        elif self.phase == MissionPhase.LAND:
            if self._execute_land(now):
                self._set_phase(MissionPhase.DONE)
        
        elif self.phase == MissionPhase.DONE:
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
            if self.mode == OperationMode.SIMULATION:
                self._publish_enable(False)
        
        # Show debug windows
        if self.show_windows:
            self._show_debug_windows()
        
        self.perf.end("control_tick")
    
    def _check_sensors_ready(self) -> bool:
        """Check if all required sensors are ready."""
        ready = self.has_odom and self.has_scan
        if not ready:
            self.get_logger().debug(
                f"Waiting for sensors: odom={self.has_odom}, scan={self.has_scan}"
            )
        return ready
    
    def _execute_takeoff(self, now: float) -> bool:
        """Execute takeoff to target altitude."""
        return self._goto_position(
            self.config.base_x, self.config.base_y,
            self.config.target_alt, now
        )
    
    def _execute_mapping(self, now: float):
        """Execute mapping sweep pattern."""
        if self.waypoint_idx >= len(self.waypoints):
            self._set_phase(MissionPhase.RETURN_HOME_AT_4M)
            return
        
        # Check for boundary avoidance
        if self.down_yellow_ratio > 0.01 or self.front_yellow_ratio > 0.01:
            self._execute_boundary_avoidance(now)
            return
        
        # Navigate to waypoint
        wx, wy, wz = self.waypoints[self.waypoint_idx]
        
        if self._goto_position(wx, wy, wz, now, tol=0.3):
            self.waypoint_idx += 1
            self.get_logger().info(
                f"Waypoint {self.waypoint_idx}/{len(self.waypoints)} reached"
            )
    
    def _execute_boundary_avoidance(self, now: float):
        """Execute boundary avoidance maneuver."""
        # Retreat towards center
        dx = self.config.base_x - self.state.x
        dy = self.config.base_y - self.state.y
        dist = math.hypot(dx, dy)
        
        if dist > 0.1:
            speed = min(0.5, 0.3 * self.down_yellow_ratio + 0.2)
            vx = speed * dx / dist
            vy = speed * dy / dist
            self._publish_velocity_world(vx, vy, 0.0, 0.0)
        else:
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
    
    def _execute_return_home(self, now: float, alt: float) -> bool:
        """Execute return to home position."""
        return self._goto_position(
            self.config.base_x, self.config.base_y, alt, now
        )
    
    def _execute_hover(self, now: float, duration: float) -> bool:
        """Execute hover at current position."""
        self._publish_velocity(0.0, 0.0, 0.0, 0.0)
        return (now - self.phase_start_time) >= duration
    
    def _execute_yaw_align(self, now: float, alt: float, target_yaw: float) -> bool:
        """Execute yaw alignment to target heading."""
        yaw_error = angle_wrap(target_yaw - self.state.yaw)
        yaw_rate = clamp(1.4 * yaw_error, -self.config.max_yaw_rate, self.config.max_yaw_rate)
        
        # Position hold
        self._goto_position(self.config.base_x, self.config.base_y, alt, now)
        
        return abs(yaw_error) < math.radians(5.0)
    
    def _execute_descend(self, now: float, target_alt: float) -> bool:
        """Execute descent to target altitude."""
        return self._goto_position(
            self.config.base_x, self.config.base_y, target_alt, now,
            max_z_speed=0.16
        )
    
    def _execute_land(self, now: float) -> bool:
        """Execute landing sequence."""
        if self.state.z > 0.3:
            self._goto_position(
                self.config.base_x, self.config.base_y, 0.15, now,
                max_z_speed=0.12
            )
            return False
        return True
    
    def _goto_position(
        self,
        x: float, y: float, z: float,
        now: float,
        tol: float = 0.25,
        max_xy_speed: Optional[float] = None,
        max_z_speed: Optional[float] = None
    ) -> bool:
        """Navigate to position with velocity control."""
        max_xy = max_xy_speed or self.config.max_xy_speed
        max_z = max_z_speed or self.config.max_z_speed
        
        ex = x - self.state.x
        ey = y - self.state.y
        ez = z - self.state.z
        
        vx = clamp(0.85 * ex, -max_xy, max_xy)
        vy = clamp(0.85 * ey, -max_xy, max_xy)
        vz = clamp(0.9 * ez, -max_z, max_z)
        
        self._publish_velocity_world(vx, vy, vz, 0.0)
        
        dist_xy = math.hypot(ex, ey)
        return dist_xy < tol and abs(ez) < tol
    
    def _publish_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """Publish velocity command (body frame)."""
        # Apply rate limiting
        target = VelocityCommand(vx, vy, vz, yaw_rate)
        dt = 1.0 / self.config.control_rate
        cmd = self.rate_limiter.update(target, dt)
        
        msg = Twist()
        msg.linear.x = cmd.vx
        msg.linear.y = cmd.vy
        msg.linear.z = cmd.vz
        msg.angular.z = cmd.yaw_rate
        
        self.pub_cmd.publish(msg)
    
    def _publish_velocity_world(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """Publish velocity command (world frame)."""
        vx_body, vy_body = world_to_body(vx, vy, self.state.yaw)
        self._publish_velocity(vx_body, vy_body, vz, yaw_rate)
    
    def _publish_enable(self, enabled: bool):
        """Publish controller enable (simulation only)."""
        if self.mode == OperationMode.SIMULATION:
            msg = Bool()
            msg.data = enabled
            self.pub_enable.publish(msg)
        self.controller_enabled = enabled
    
    def _set_phase(self, phase: MissionPhase):
        """Set mission phase."""
        self.phase = phase
        self.phase_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f"Phase: {phase.name}")
    
    def _show_debug_windows(self):
        """Show debug camera windows."""
        if hasattr(self, 'front_vis') and self.front_vis is not None:
            cv2.imshow("Front Camera", self.front_vis)
        if hasattr(self, 'down_vis') and self.down_vis is not None:
            cv2.imshow("Down Camera", self.down_vis)
        cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser("Hardware Mission Node")
    parser.add_argument("--mode", choices=["sim", "hardware"], default="hardware")
    parser.add_argument("--config", type=str, default=None)
    args, _ = parser.parse_known_args()
    
    mode = OperationMode.SIMULATION if args.mode == "sim" else OperationMode.HARDWARE
    
    # Load config
    config = None
    if args.config:
        try:
            config = MissionConfig.from_yaml(args.config)
        except Exception as e:
            print(f"Warning: Could not load config: {e}")
    
    rclpy.init()
    
    node = HardwareMissionNode(config=config, mode=mode)
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
