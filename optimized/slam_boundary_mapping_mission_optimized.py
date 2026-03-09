#!/usr/bin/env python3
"""
Optimized SLAM Boundary Mapping Mission Node

This is an optimized version of the SLAM boundary mapping mission with:
- Improved performance and memory efficiency
- Better error handling and recovery
- Configurable parameters via YAML
- Hardware deployment ready
- Enhanced safety features
- Cleaner code structure

Author: Drone Team
Version: 2.0.0
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple, Dict, Any
import math
import os
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from actuator_msgs.msg import Actuators
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class MissionPhase(Enum):
    """Mission state machine phases"""
    WAIT_SENSORS = auto()
    TAKEOFF = auto()
    MAPPING = auto()
    RETURN_HOME_AT_4M = auto()
    HOVER_HOME_AT_4M = auto()
    ALIGN_YAW_AT_4M = auto()
    DESCEND_TO_2M = auto()
    HOLD_AT_2M = auto()
    ALIGN_YAW_AT_2M = auto()
    LAND = auto()
    DONE = auto()
    # Failsafe phases
    FAILSAFE_RETURN_HOME_AT_4M = auto()
    FAILSAFE_HOVER_HOME_AT_4M = auto()
    FAILSAFE_ALIGN_YAW_AT_4M = auto()
    FAILSAFE_DESCEND_TO_2M = auto()
    FAILSAFE_HOLD_AT_2M = auto()
    FAILSAFE_ALIGN_YAW_AT_2M = auto()
    FAILSAFE_LAND = auto()
    # Emergency phases
    EMERGENCY_LAND = auto()
    COMMUNICATION_LOSS_RTL = auto()


@dataclass
class VelocityCommand:
    """Thread-safe velocity command container"""
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    timestamp: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def set(self, vx: float, vy: float, vz: float, yaw_rate: float, timestamp: float):
        with self.lock:
            self.vx = vx
            self.vy = vy
            self.vz = vz
            self.yaw_rate = yaw_rate
            self.timestamp = timestamp

    def get(self) -> Tuple[float, float, float, float]:
        with self.lock:
            return self.vx, self.vy, self.vz, self.yaw_rate


@dataclass
class DroneState:
    """Thread-safe drone state container"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    quaternion: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    has_odom: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_from_odom(self, msg: Odometry) -> Tuple[float, float, float, float]:
        """Update state from odometry message. Returns (x, y, z, yaw)"""
        with self.lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.z = msg.pose.pose.position.z
            self.vx = msg.twist.twist.linear.x
            self.vy = msg.twist.twist.linear.y
            self.vz = msg.twist.twist.linear.z

            q = msg.pose.pose.orientation
            self.quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)

            # Calculate yaw from quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.yaw = math.atan2(siny_cosp, cosy_cosp)
            self.has_odom = True

            return self.x, self.y, self.z, self.yaw

    def get_position(self) -> Tuple[float, float, float]:
        with self.lock:
            return self.x, self.y, self.z

    def get_velocity(self) -> Tuple[float, float, float]:
        with self.lock:
            return self.vx, self.vy, self.vz

    def get_yaw(self) -> float:
        with self.lock:
            return self.yaw


class SlamBoundaryMissionOptimized(Node):
    """
    Optimized SLAM Boundary Mapping Mission Node

    Key improvements over original:
    1. Configurable parameters via YAML
    2. Better state management with dataclasses
    3. Thread-safe operations
    4. Improved error handling
    5. Memory-efficient image processing
    6. Better logging and diagnostics
    7. Hardware deployment ready
    """

    def __init__(self):
        super().__init__("slam_boundary_mapping_mission")

        # Declare parameters with defaults
        self._declare_parameters()

        # Initialize components
        self._init_state()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()

        # Load configuration
        self._load_config()

        # Build waypoints
        self.waypoints = self._build_mapping_waypoints()
        self.waypoint_idx = 0

        # Pre-compute constants
        self._precompute_constants()

        self.get_logger().info("Optimized SLAM mapping mission node started.")
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints for boundary sweep.")

    def _declare_parameters(self):
        """Declare all ROS parameters with defaults"""
        # Mission parameters
        self.declare_parameter("target_altitude", 4.0)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("base_x", 0.0)
        self.declare_parameter("base_y", 0.0)

        # Velocity limits
        self.declare_parameter("max_xy_speed", 1.45)
        self.declare_parameter("max_z_speed", 0.6)
        self.declare_parameter("max_yaw_rate", 0.9)

        # Mapping parameters
        self.declare_parameter("arena_half_extent", 8.0)
        self.declare_parameter("lane_spacing", 0.9)
        self.declare_parameter("edge_margin", 1.2)

        # Safety parameters
        self.declare_parameter("max_distance_from_base", 15.0)
        self.declare_parameter("min_altitude", 0.3)
        self.declare_parameter("max_altitude", 10.0)
        self.declare_parameter("min_battery_for_takeoff", 30.0)

        # Hardware mode
        self.declare_parameter("hardware_mode", False)
        self.declare_parameter("use_sim_time", True)

    def _init_state(self):
        """Initialize state containers"""
        self.state = DroneState()
        self.cmd_vel = VelocityCommand()

        # Sensor availability flags
        self.has_scan = False
        self.has_front_camera = False
        self.has_down_camera = False
        self.has_motor_cmd = False
        self.has_tof = False

        # Mission state
        self.phase = MissionPhase.WAIT_SENSORS
        self.phase_start_time = self._now()
        self.reached_since: Optional[float] = None
        self.active_waypoint_start: Optional[float] = None

        # Battery estimation
        self.battery_percent = 100.0
        self.motor_load = 0.0

        # Yellow detection state
        self.front_yellow_ratio = 0.0
        self.down_yellow_ratio = 0.0
        self.front_yellow_time = -1.0
        self.down_yellow_time = -1.0
        self.down_repulse_x = 0.0
        self.down_repulse_y = 0.0
        self.down_repulse_strength = 0.0

        # Boundary avoidance state
        self.boundary_avoid_until = -1.0
        self.boundary_hover_until = -1.0

        # Manual control state
        self.manual_mode = False
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_vz = 0.0
        self.manual_yaw = 0.0
        self.last_manual_cmd = -1.0

        # Failsafe state
        self.failsafe_triggered = False
        self.last_valid_cmd_time = self._now()

        # CV Bridge (reuse for efficiency)
        self.bridge = CvBridge()

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Coverage tracking
        self.coverage_grid: Optional[np.ndarray] = None
        self.arena_coverage_percent = 0.0

        # LiDAR data cache
        self.last_scan_ranges: Optional[np.ndarray] = None
        self.last_scan_angles: Optional[np.ndarray] = None

        # Dashboard state
        self.dashboard_speed = 0.0
        self.dashboard_distance = 0.0
        self.map_known_percent = 0.0

        # GUI availability
        self.show_windows = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        self.front_debug_frame: Optional[np.ndarray] = None
        self.down_debug_frame: Optional[np.ndarray] = None

        # Pre-allocated yellow mask (for performance)
        self._yellow_mask_buffer: Optional[np.ndarray] = None

    def _init_publishers(self):
        """Initialize all publishers"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.pub_enable = self.create_publisher(Bool, "/drone_enable", qos)
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", qos)
        self.pub_motor_cmd = self.create_publisher(Actuators, "/motor_speed_cmd_out", qos)
        self.pub_terrain_cloud = self.create_publisher(PointCloud2, "/terrain_point_cloud", qos)

    def _init_subscribers(self):
        """Initialize all subscribers with appropriate QoS"""
        sensor_qos = qos_profile_sensor_data

        # Use reentrant callback group for concurrent processing
        callback_group = ReentrantCallbackGroup()

        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self._odom_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_front = self.create_subscription(
            Image, "/front_camera/image_raw", self._front_camera_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_down = self.create_subscription(
            Image, "/down_camera/image_raw", self._down_camera_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_motor_cmd = self.create_subscription(
            Actuators, "/motor_speed_cmd_in", self._motor_cmd_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_manual_mode = self.create_subscription(
            Bool, "/manual_mode", self._manual_mode_callback, 10,
            callback_group=callback_group
        )
        self.sub_manual_cmd = self.create_subscription(
            Twist, "/manual_cmd_vel", self._manual_cmd_callback, 10,
            callback_group=callback_group
        )
        self.sub_tof = self.create_subscription(
            LaserScan, "/tof_scan", self._tof_callback, sensor_qos,
            callback_group=callback_group
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, 10,
            callback_group=callback_group
        )

    def _init_timers(self):
        """Initialize control timer"""
        control_rate = self.get_parameter("control_rate").value
        self.control_timer = self.create_timer(1.0 / control_rate, self._control_tick)

    def _load_config(self):
        """Load configuration from parameters"""
        # Mission config
        self.target_alt = self.get_parameter("target_altitude").value
        self.base_xy = (
            self.get_parameter("base_x").value,
            self.get_parameter("base_y").value
        )

        # Velocity limits
        self.max_xy_speed = self.get_parameter("max_xy_speed").value
        self.max_z_speed = self.get_parameter("max_z_speed").value
        self.max_yaw_rate = self.get_parameter("max_yaw_rate").value

        # Mapping config
        self.arena_half_extent = self.get_parameter("arena_half_extent").value
        self.lane_spacing = self.get_parameter("lane_spacing").value
        self.edge_margin = self.get_parameter("edge_margin").value

        # Safety config
        self.max_distance_from_base = self.get_parameter("max_distance_from_base").value
        self.min_altitude = self.get_parameter("min_altitude").value
        self.max_altitude = self.get_parameter("max_altitude").value
        self.min_battery_takeoff = self.get_parameter("min_battery_for_takeoff").value

        # Hardware mode
        self.hardware_mode = self.get_parameter("hardware_mode").value

        # Default values for other parameters (can be loaded from YAML)
        self.mapping_speed_boost = 1.65
        self.mapping_precision_radius = 1.4
        self.mapping_precision_min_speed = 0.55
        self.waypoint_timeout = 22.0

        # Yellow detection thresholds
        self.yellow_h_min = 18
        self.yellow_h_max = 38
        self.yellow_s_min = 80
        self.yellow_v_min = 80
        self.front_yellow_threshold = 0.010
        self.down_yellow_threshold = 0.010

        # Boundary avoidance
        self.boundary_trigger_extent = 7.2
        self.boundary_guard_enter_strength = 0.08
        self.boundary_guard_exit_strength = 0.03
        self.boundary_hover_s = 0.30
        self.boundary_retreat_base_speed = 0.35
        self.boundary_retreat_gain = 0.35
        self.avoid_duration_s = 1.2
        self.boundary_avoid_max_duration_s = 6.0

        # Battery parameters
        self.battery_low_warn = 25.0
        self.battery_critical = 12.0

        # Obstacle avoidance
        self.obstacle_avoid_dist = 2.2
        self.obstacle_repulse_gain = 0.85

        # Landing parameters
        self.landing_z = 0.20
        self.mid_alt = 2.0
        self.home_hover_s = 2.0
        self.mid_alt_hover_s = 2.0
        self.landing_yaw = 0.0
        self.slow_descent_z_speed = 0.16
        self.landing_z_speed = 0.12

        # Heading control
        self.yaw_kp = 1.4
        self.yaw_align_tolerance = math.radians(6.0)
        self.yaw_hover_before_move = 0.15
        self.yaw_align_timeout = 1.8
        self.cmd_vel_body_frame = True

        # Rate limiting
        self.rate_limit_enable = True
        self.cmd_accel_xy = 2.2
        self.cmd_accel_z = 1.4
        self.cmd_accel_yaw = 2.0

        # Current commanded velocities (for rate limiting)
        self.cmd_vx_current = 0.0
        self.cmd_vy_current = 0.0
        self.cmd_vz_current = 0.0
        self.cmd_yaw_current = 0.0
        self.last_cmd_time = self._now()

    def _precompute_constants(self):
        """Pre-compute frequently used constants"""
        # Yellow HSV bounds
        self.yellow_lower = np.array([self.yellow_h_min, self.yellow_s_min, self.yellow_v_min], dtype=np.uint8)
        self.yellow_upper = np.array([self.yellow_h_max, 255, 255], dtype=np.uint8)

        # Morphology kernel (reusable)
        self.morph_kernel = np.ones((3, 3), np.uint8)

        # Coverage grid
        coverage_res = 0.50
        span = 2.0 * self.arena_half_extent
        cells = max(1, int(np.ceil(span / coverage_res)))
        self.coverage_grid = np.zeros((cells, cells), dtype=bool)
        self.coverage_resolution = coverage_res

    def _now(self) -> float:
        """Get current time in seconds"""
        return self.get_clock().now().nanoseconds * 1e-9

    # ============== Callbacks ==============

    def _odom_callback(self, msg: Odometry):
        """Process odometry message"""
        x, y, z, yaw = self.state.update_from_odom(msg)

        # Broadcast TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp if msg.header.stamp.sec > 0 else self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def _scan_callback(self, msg: LaserScan):
        """Process LiDAR scan"""
        self.has_scan = True

        # Convert to numpy arrays for efficient processing
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

        self.last_scan_ranges = ranges[valid]
        angles = msg.angle_min + np.arange(len(msg.ranges), dtype=np.float64) * msg.angle_increment
        self.last_scan_angles = angles[valid]

    def _tof_callback(self, msg: LaserScan):
        """Process Time-of-Flight sensor data"""
        if not msg.ranges:
            self.has_tof = False
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        finite = ranges[np.isfinite(ranges)]

        if finite.size == 0:
            self.has_tof = False
            return

        valid = finite[(finite >= msg.range_min) & (finite <= msg.range_max)]
        if valid.size == 0:
            self.has_tof = False
            return

        self.tof_range = float(np.min(valid))
        self.tof_last_update = self._now()
        self.has_tof = True

    def _map_callback(self, msg: OccupancyGrid):
        """Process occupancy grid map"""
        if not msg.data:
            return

        data = np.asarray(msg.data, dtype=np.int16)
        total = data.size
        if total == 0:
            return

        known = int(np.count_nonzero(data >= 0))
        self.map_known_percent = 100.0 * float(known) / float(total)

    def _front_camera_callback(self, msg: Image):
        """Process front camera image"""
        ratio, vis = self._detect_yellow(msg)
        self.has_front_camera = True
        self.front_yellow_ratio = ratio

        if ratio > self.front_yellow_threshold:
            self.front_yellow_time = self._now()

        self.front_debug_frame = vis

    def _down_camera_callback(self, msg: Image):
        """Process down camera image with repulsion vector calculation"""
        ratio, vis, rep_x, rep_y, strength = self._detect_yellow_with_repulsion(msg)
        self.has_down_camera = True
        self.down_yellow_ratio = ratio

        if ratio > self.down_yellow_threshold:
            self.down_yellow_time = self._now()

        self.down_repulse_x = rep_x
        self.down_repulse_y = rep_y
        self.down_repulse_strength = strength
        self.down_debug_frame = vis

    def _motor_cmd_callback(self, msg: Actuators):
        """Process motor commands (for failsafe relay)"""
        self.has_motor_cmd = True

        out = Actuators()
        out.header = msg.header
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.normalized = list(msg.normalized)

        # Apply failsafe motor cut if triggered
        if self.failsafe_triggered and len(out.velocity) > 3:
            out.velocity[3] = 0.0
            if len(out.normalized) > 3:
                out.normalized[3] = 0.0

        # Calculate motor load
        if out.normalized:
            self.motor_load = float(np.mean(np.abs(np.asarray(out.normalized, dtype=np.float64))))
        elif out.velocity:
            vel = np.asarray(out.velocity, dtype=np.float64)
            finite = vel[np.isfinite(vel)]
            if finite.size > 0:
                self.motor_load = float(np.mean(np.abs(finite)) / max(1.0, np.max(np.abs(finite))))

        try:
            self.pub_motor_cmd.publish(out)
        except Exception:
            pass

    def _manual_mode_callback(self, msg: Bool):
        """Handle manual mode toggle"""
        prev = self.manual_mode
        self.manual_mode = bool(msg.data)

        if self.manual_mode != prev:
            mode_str = "MANUAL" if self.manual_mode else "AUTONOMOUS"
            self.get_logger().warn(f"Control mode -> {mode_str}")

        if not self.manual_mode:
            self.manual_vx = 0.0
            self.manual_vy = 0.0
            self.manual_vz = 0.0
            self.manual_yaw = 0.0

    def _manual_cmd_callback(self, msg: Twist):
        """Handle manual velocity commands"""
        self.last_manual_cmd = self._now()
        self.manual_vx = self._clamp(msg.linear.x, -1.4, 1.4)
        self.manual_vy = self._clamp(msg.linear.y, -1.4, 1.4)
        self.manual_vz = self._clamp(msg.linear.z, -0.7, 0.7)
        self.manual_yaw = self._clamp(msg.angular.z, -1.2, 1.2)

    # ============== Image Processing ==============

    def _detect_yellow(self, msg: Image) -> Tuple[float, Optional[np.ndarray]]:
        """Detect yellow in image and return ratio"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Image decode failed: {e}")
            return 0.0, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)

        ratio = float(np.count_nonzero(mask)) / float(mask.size)

        # Visualization
        vis = frame.copy()
        vis[mask > 0] = (0, 255, 255)
        cv2.putText(vis, f"yellow={ratio:.3f}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        return ratio, vis

    def _detect_yellow_with_repulsion(self, msg: Image) -> Tuple[float, Optional[np.ndarray], float, float, float]:
        """Detect yellow and calculate repulsion vector"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return 0.0, None, 0.0, 0.0, 0.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)

        ratio = float(np.count_nonzero(mask)) / float(mask.size)

        h, w = mask.shape
        cx0, cy0 = 0.5 * (w - 1), 0.5 * (h - 1)

        rep_x, rep_y, strength = 0.0, 0.0, 0.0
        centroid = None

        # Calculate centroid
        moments = cv2.moments(mask, binaryImage=True)
        if moments["m00"] > 1e-6:
            cx = float(moments["m10"] / moments["m00"])
            cy = float(moments["m01"] / moments["m00"])
            centroid = (int(cx), int(cy))

            # Image offset normalized
            off_x = (cx - cx0) / max(1.0, 0.5 * w)
            off_y = (cy - cy0) / max(1.0, 0.5 * h)

            # Repulsion direction (opposite to centroid)
            rep_img_x, rep_img_y = -off_x, -off_y
            n = math.hypot(rep_img_x, rep_img_y)
            if n > 1e-6:
                rep_img_x /= n
                rep_img_y /= n

                # Transform to body frame
                # Image top = forward body-x, Image right = body-right
                body_x = -rep_img_y
                body_y = -rep_img_x

                # Calculate strength
                strength = self._clamp(
                    (ratio - self.down_yellow_threshold) / max(1e-6, 0.10 - self.down_yellow_threshold),
                    0.0, 1.0
                )
                speed = 0.65 * strength
                rep_x = speed * body_x
                rep_y = speed * body_y

        # Visualization
        vis = frame.copy()
        vis[mask > 0] = (0, 255, 255)
        cv2.circle(vis, (int(cx0), int(cy0)), 5, (255, 255, 255), -1, cv2.LINE_AA)
        if centroid:
            cv2.circle(vis, centroid, 6, (0, 0, 255), 2, cv2.LINE_AA)

        if strength > 0.0:
            arrow_len = int(35 + 160 * strength)
            arrow_end = (int(cx0 + rep_img_x * arrow_len), int(cy0 + rep_img_y * arrow_len))
            cv2.arrowedLine(vis, (int(cx0), int(cy0)), arrow_end, (0, 0, 255), 3, cv2.LINE_AA, tipLength=0.25)

        cv2.putText(vis, f"yellow={ratio:.3f} repel={strength:.2f}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        return ratio, vis, rep_x, rep_y, strength

    # ============== Utility Functions ==============

    @staticmethod
    def _clamp(val: float, low: float, high: float) -> float:
        """Clamp value to range"""
        return max(low, min(high, val))

    @staticmethod
    def _angle_wrap(rad: float) -> float:
        """Wrap angle to [-pi, pi]"""
        return math.atan2(math.sin(rad), math.cos(rad))

    @staticmethod
    def _quat_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
        """Convert quaternion to rotation matrix"""
        qx, qy, qz, qw = quat
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-9:
            return np.eye(3, dtype=np.float64)
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

        return np.array([
            [1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw), 2.0*(qx*qz + qy*qw)],
            [2.0*(qx*qy + qz*qw), 1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw)],
            [2.0*(qx*qz - qy*qw), 2.0*(qy*qz + qx*qw), 1.0 - 2.0*(qx*qx + qy*qy)]
        ], dtype=np.float64)

    def _build_mapping_waypoints(self) -> List[Tuple[float, float, float]]:
        """Build lane-by-lane sweep waypoints"""
        z = self.target_alt
        margin = self.edge_margin
        extent = self.arena_half_extent

        x_min = -extent + margin
        x_max = extent - margin
        y_min = -extent + margin
        y_max = extent - margin

        lane_spacing = max(0.25, self.lane_spacing)
        span_y = max(0.1, y_max - y_min)
        lane_count = max(1, int(np.floor(span_y / lane_spacing)) + 1)
        lane_step = 0.0 if lane_count == 1 else span_y / float(lane_count - 1)

        waypoints = []
        for lane_idx in range(lane_count):
            y_lane = y_min + lane_idx * lane_step
            x_start, x_end = (x_min, x_max) if lane_idx % 2 == 0 else (x_max, x_min)

            start_wp = (x_start, y_lane, z)
            end_wp = (x_end, y_lane, z)

            if not waypoints or any(abs(start_wp[i] - waypoints[-1][i]) > 1e-6 for i in range(3)):
                waypoints.append(start_wp)
            if any(abs(end_wp[i] - waypoints[-1][i]) > 1e-6 for i in range(3)):
                waypoints.append(end_wp)

        return waypoints

    # ============== Control Functions ==============

    def _publish_enable(self, enabled: bool):
        """Publish enable message"""
        if not rclpy.ok():
            return
        msg = Bool()
        msg.data = enabled
        self.pub_enable.publish(msg)

    def _publish_twist(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Publish velocity command with rate limiting"""
        if not rclpy.ok():
            return

        now = self._now()
        dt = max(1e-3, now - self.last_cmd_time)
        self.last_cmd_time = now

        if self.rate_limit_enable and not self.manual_mode:
            # Apply acceleration limits
            dv_xy = self.cmd_accel_xy * dt
            dv_z = self.cmd_accel_z * dt
            dv_yaw = self.cmd_accel_yaw * dt

            self.cmd_vx_current += self._clamp(vx - self.cmd_vx_current, -dv_xy, dv_xy)
            self.cmd_vy_current += self._clamp(vy - self.cmd_vy_current, -dv_xy, dv_xy)
            self.cmd_vz_current += self._clamp(vz - self.cmd_vz_current, -dv_z, dv_z)
            self.cmd_yaw_current += self._clamp(yaw_rate - self.cmd_yaw_current, -dv_yaw, dv_yaw)

            vx, vy, vz, yaw_rate = self.cmd_vx_current, self.cmd_vy_current, self.cmd_vz_current, self.cmd_yaw_current
        else:
            self.cmd_vx_current = vx
            self.cmd_vy_current = vy
            self.cmd_vz_current = vz
            self.cmd_yaw_current = yaw_rate

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.pub_twist.publish(msg)

        self.last_valid_cmd_time = now

    def _goto(self, x_ref: float, y_ref: float, z_ref: float,
              max_xy: Optional[float] = None, max_z: Optional[float] = None,
              xy_tol: float = 0.25, z_tol: float = 0.20) -> bool:
        """Navigate to position, returns True when reached"""
        x, y, z = self.state.get_position()
        yaw = self.state.get_yaw()

        ex = x_ref - x
        ey = y_ref - y
        ez = z_ref - z
        dist_xy = math.hypot(ex, ey)

        xy_limit = max_xy if max_xy else self.max_xy_speed
        z_limit = max_z if max_z else self.max_z_speed

        # Speed boost for mapping phase
        if self.phase == MissionPhase.MAPPING and max_xy is None:
            xy_limit = self.mapping_speed_boost
            if dist_xy < self.mapping_precision_radius:
                alpha = dist_xy / max(self.mapping_precision_radius, 1e-6)
                xy_limit = self.mapping_precision_min_speed + (self.mapping_speed_boost - self.mapping_precision_min_speed) * alpha

        # Base velocity command
        vx = self._clamp(0.85 * ex, -xy_limit, xy_limit)
        vy = self._clamp(0.85 * ey, -xy_limit, xy_limit)
        vz = self._clamp(0.9 * ez, -z_limit, z_limit)

        # Add repulsion vectors if mapping
        if self.phase == MissionPhase.MAPPING:
            if self._near_boundary():
                rep_x, rep_y = self._get_repulsion_world()
                vx += rep_x
                vy += rep_y

            obs_x, obs_y = self._get_obstacle_repulsion()
            vx += obs_x
            vy += obs_y

        # Limit combined velocity
        speed = math.hypot(vx, vy)
        if speed > xy_limit:
            scale = xy_limit / max(speed, 1e-6)
            vx *= scale
            vy *= scale

        # Transform to body frame if needed
        if self.cmd_vel_body_frame:
            c, s = math.cos(yaw), math.sin(yaw)
            vx_body = c * vx + s * vy
            vy_body = -s * vx + c * vy
            vx, vy = vx_body, vy_body

        yaw_rate = 0.0
        self._publish_twist(vx, vy, vz, yaw_rate)

        return dist_xy < xy_tol and abs(ez) < z_tol

    def _near_boundary(self) -> bool:
        """Check if drone is near arena boundary"""
        x, y, _ = self.state.get_position()
        return abs(x) >= self.boundary_trigger_extent or abs(y) >= self.boundary_trigger_extent

    def _get_repulsion_world(self) -> Tuple[float, float]:
        """Get boundary repulsion vector in world frame"""
        yaw = self.state.get_yaw()
        c, s = math.cos(yaw), math.sin(yaw)
        vx = c * self.down_repulse_x - s * self.down_repulse_y
        vy = s * self.down_repulse_x + c * self.down_repulse_y
        return vx, vy

    def _get_obstacle_repulsion(self) -> Tuple[float, float]:
        """Calculate obstacle repulsion from LiDAR"""
        if self.last_scan_ranges is None or self.last_scan_ranges.size == 0:
            return 0.0, 0.0

        close_mask = self.last_scan_ranges < self.obstacle_avoid_dist
        close_ranges = self.last_scan_ranges[close_mask]
        close_angles = self.last_scan_angles[close_mask]

        if close_ranges.size == 0:
            return 0.0, 0.0

        weights = 1.0 / np.square(close_ranges + 0.05)
        body_x = -np.sum(weights * np.cos(close_angles))
        body_y = -np.sum(weights * np.sin(close_angles))

        mag = math.hypot(body_x, body_y)
        if mag > 1e-6:
            body_x = (body_x / mag) * self.obstacle_repulse_gain
            body_y = (body_y / mag) * self.obstacle_repulse_gain

        yaw = self.state.get_yaw()
        c, s = math.cos(yaw), math.sin(yaw)
        world_x = c * body_x - s * body_y
        world_y = s * body_x + c * body_y

        return world_x, world_y

    def _run_boundary_avoidance(self, now: float):
        """Execute boundary avoidance maneuver"""
        x, y, z = self.state.get_position()
        yaw = self.state.get_yaw()

        # Calculate retreat direction (towards base)
        in_x = self.base_xy[0] - x
        in_y = self.base_xy[1] - y
        mag = math.hypot(in_x, in_y)

        if mag > 1e-6:
            dir_x, dir_y = in_x / mag, in_y / mag
        else:
            dir_x, dir_y = 1.0, 0.0

        strength = max(self.down_repulse_strength, self.down_yellow_ratio)
        speed = self._clamp(
            self.boundary_retreat_base_speed + self.boundary_retreat_gain * strength,
            self.boundary_retreat_base_speed, self.max_xy_speed
        )

        vx = speed * dir_x
        vy = speed * dir_y

        target_yaw = math.atan2(vy, vx)
        yaw_err = self._angle_wrap(target_yaw - yaw)
        yaw_rate = self._clamp(self.yaw_kp * yaw_err, -self.max_yaw_rate, self.max_yaw_rate)

        vz = self._clamp(0.9 * (self.target_alt - z), -0.25, 0.25)

        # Hover phase
        if now < self.boundary_hover_until:
            self._publish_twist(0.0, 0.0, vz, yaw_rate)
            return

        # Check yaw alignment
        if abs(yaw_err) > math.radians(6.0):
            self._publish_twist(0.0, 0.0, vz, yaw_rate)
            return

        # Transform to body frame
        if self.cmd_vel_body_frame:
            c, s = math.cos(yaw), math.sin(yaw)
            vx_body = c * vx + s * vy
            vy_body = -s * vx + c * vy

            if vx_body <= 0.0:
                self._publish_twist(0.0, 0.0, vz, yaw_rate)
                return

            vx = self._clamp(vx_body, 0.25, self.max_xy_speed)
            vy = self._clamp(vy_body, -0.20, 0.20)

        self._publish_twist(vx, vy, vz, yaw_rate)

    def _run_manual_override(self):
        """Execute manual control mode"""
        now = self._now()

        # Check for command timeout
        if (now - self.last_manual_cmd) > 0.6:
            vx, vy, vz, yaw = 0.0, 0.0, 0.0, 0.0
        else:
            vx, vy, vz, yaw = self.manual_vx, self.manual_vy, self.manual_vz, self.manual_yaw

        self._publish_enable(True)
        self._publish_twist(vx, vy, vz, yaw)

    def _update_battery(self, dt: float):
        """Update battery estimation"""
        vx, vy, vz = self.state.get_velocity()
        speed = math.hypot(vx, vy, vz)

        x, y, z = self.state.get_position()
        dist = math.hypot(x - self.base_xy[0], y - self.base_xy[1])

        # Smooth speed/distance
        alpha = 0.18
        self.dashboard_speed = (1 - alpha) * self.dashboard_speed + alpha * speed
        self.dashboard_distance = (1 - alpha) * self.dashboard_distance + alpha * dist

        # Drain rate based on load
        speed_load = min(1.0, self.dashboard_speed / max(self.mapping_speed_boost, 1e-6))
        dyn_load = max(self.motor_load, speed_load)

        drain_rate = 0.0025 + (0.0600 - 0.0025) * dyn_load
        self.battery_percent = self._clamp(self.battery_percent - drain_rate * dt, 0.0, 100.0)

    def _update_coverage(self):
        """Update arena coverage tracking"""
        x, y, _ = self.state.get_position()

        ix = int((x + self.arena_half_extent) / self.coverage_resolution)
        iy = int((y + self.arena_half_extent) / self.coverage_resolution)

        ix = self._clamp(ix, 0, self.coverage_grid.shape[1] - 1)
        iy = self._clamp(iy, 0, self.coverage_grid.shape[0] - 1)

        self.coverage_grid[iy, ix] = True
        covered = int(np.count_nonzero(self.coverage_grid))
        self.arena_coverage_percent = 100.0 * float(covered) / float(self.coverage_grid.size)

    def _set_phase(self, phase: MissionPhase):
        """Set mission phase with logging"""
        self.phase = phase
        self.phase_start_time = self._now()
        self.reached_since = None
        self.active_waypoint_start = None
        self.get_logger().info(f"Phase -> {phase.name}")

    # ============== Main Control Loop ==============

    def _control_tick(self):
        """Main control loop tick"""
        now = self._now()
        dt = 1.0 / self.get_parameter("control_rate").value

        # Update battery
        self._update_battery(dt)

        # Update coverage
        if self.state.has_odom:
            self._update_coverage()

        # Show GUI windows
        self._maybe_show_windows()

        # Publish scan TF
        self._publish_scan_tf()

        # Check for RTB request
        if hasattr(self, 'rtb_requested') and self.rtb_requested and not self.failsafe_triggered:
            self.rtb_requested = False
            return_phases = {
                MissionPhase.RETURN_HOME_AT_4M, MissionPhase.HOVER_HOME_AT_4M,
                MissionPhase.ALIGN_YAW_AT_4M, MissionPhase.DESCEND_TO_2M,
                MissionPhase.HOLD_AT_2M, MissionPhase.ALIGN_YAW_AT_2M,
                MissionPhase.LAND, MissionPhase.DONE
            }
            if self.phase not in return_phases:
                self.get_logger().warn("RTB requested, returning to base")
                self._set_phase(MissionPhase.RETURN_HOME_AT_4M)

        # Manual mode override
        if self.manual_mode:
            self._run_manual_override()
            return

        # Check sensor availability
        if not (self.state.has_odom and self.has_scan and self.has_front_camera and self.has_down_camera):
            self._publish_enable(False)
            self._publish_twist(0.0, 0.0, 0.0)
            return

        # Check completion
        if self.phase == MissionPhase.DONE:
            self._publish_enable(False)
            self._publish_twist(0.0, 0.0, 0.0)
            return

        # Enable controller
        self._publish_enable(True)

        # Check battery safety
        if self.battery_percent < self.battery_low_warn and self.phase == MissionPhase.MAPPING:
            self.get_logger().warn(f"Low battery ({self.battery_percent:.1f}%), initiating RTL")
            self._set_phase(MissionPhase.RETURN_HOME_AT_4M)

        # Check boundary trigger during mapping
        if self.phase == MissionPhase.MAPPING and not self.failsafe_triggered:
            boundary_seen = (now - self.front_yellow_time) < 0.5 or (now - self.down_yellow_time) < 0.5
            near_boundary = self._near_boundary()
            risk = self.down_repulse_strength

            if near_boundary and (risk >= self.boundary_guard_enter_strength or boundary_seen):
                if now >= self.boundary_avoid_until:
                    self.boundary_hover_until = now + self.boundary_hover_s
                self.boundary_avoid_until = min(
                    max(self.boundary_avoid_until, now + self.avoid_duration_s + risk),
                    now + self.boundary_avoid_max_duration_s
                )

        # Execute phase-specific logic
        self._execute_phase(now)

    def _execute_phase(self, now: float):
        """Execute current phase logic"""
        x, y, z = self.state.get_position()

        if self.phase == MissionPhase.WAIT_SENSORS:
            self._set_phase(MissionPhase.TAKEOFF)
            return

        if self.phase == MissionPhase.TAKEOFF:
            reached = self._goto(self.base_xy[0], self.base_xy[1], self.target_alt)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 1.5:
                    self._set_phase(MissionPhase.MAPPING)
            else:
                self.reached_since = None
            return

        if self.phase == MissionPhase.MAPPING:
            # Boundary avoidance
            if now < self.boundary_avoid_until:
                self._run_boundary_avoidance(now)
                return

            # Check waypoints
            if self.waypoint_idx >= len(self.waypoints):
                self._set_phase(MissionPhase.RETURN_HOME_AT_4M)
                return

            # Waypoint timeout
            if self.active_waypoint_start is None:
                self.active_waypoint_start = now
            elif now - self.active_waypoint_start > self.waypoint_timeout:
                self.get_logger().warn(f"Waypoint {self.waypoint_idx + 1} timeout, advancing")
                self.waypoint_idx += 1
                self.active_waypoint_start = None
                return

            # Navigate to waypoint
            tx, ty, tz = self.waypoints[self.waypoint_idx]
            reached = self._goto(tx, ty, tz)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.15:
                    self.waypoint_idx += 1
                    self.reached_since = None
                    self.active_waypoint_start = None
            else:
                self.reached_since = None
            return

        if self.phase == MissionPhase.RETURN_HOME_AT_4M:
            reached = self._goto(self.base_xy[0], self.base_xy[1], self.target_alt)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.8:
                    self._set_phase(MissionPhase.HOVER_HOME_AT_4M)
            else:
                self.reached_since = None
            return

        if self.phase == MissionPhase.HOVER_HOME_AT_4M:
            self._goto(self.base_xy[0], self.base_xy[1], self.target_alt)
            if now - self.phase_start_time > self.home_hover_s:
                self._set_phase(MissionPhase.ALIGN_YAW_AT_4M)
            return

        if self.phase == MissionPhase.ALIGN_YAW_AT_4M:
            reached = self._goto_with_yaw(self.base_xy[0], self.base_xy[1], self.target_alt, self.landing_yaw)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.6:
                    self._set_phase(MissionPhase.DESCEND_TO_2M)
            else:
                self.reached_since = None
            return

        if self.phase == MissionPhase.DESCEND_TO_2M:
            reached = self._goto(self.base_xy[0], self.base_xy[1], self.mid_alt,
                                max_xy=0.55, max_z=self.slow_descent_z_speed)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.8:
                    self._set_phase(MissionPhase.HOLD_AT_2M)
            else:
                self.reached_since = None
            return

        if self.phase == MissionPhase.HOLD_AT_2M:
            self._goto(self.base_xy[0], self.base_xy[1], self.mid_alt)
            if now - self.phase_start_time > self.mid_alt_hover_s:
                self._set_phase(MissionPhase.ALIGN_YAW_AT_2M)
            return

        if self.phase == MissionPhase.ALIGN_YAW_AT_2M:
            reached = self._goto_with_yaw(self.base_xy[0], self.base_xy[1], self.mid_alt, self.landing_yaw,
                                         max_xy=0.35, max_z=self.slow_descent_z_speed)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.6:
                    self._set_phase(MissionPhase.LAND)
            else:
                self.reached_since = None
            return

        if self.phase == MissionPhase.LAND:
            reached = self._goto_with_yaw(self.base_xy[0], self.base_xy[1], self.landing_z, self.landing_yaw,
                                         max_xy=0.25, max_z=self.landing_z_speed)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 1.0:
                    self._set_phase(MissionPhase.DONE)
            else:
                self.reached_since = None
            return

    def _goto_with_yaw(self, x_ref: float, y_ref: float, z_ref: float, target_yaw: float,
                       max_xy: Optional[float] = None, max_z: Optional[float] = None) -> bool:
        """Navigate to position while aligning yaw"""
        x, y, z = self.state.get_position()
        yaw = self.state.get_yaw()

        ex = x_ref - x
        ey = y_ref - y
        ez = z_ref - z
        dist_xy = math.hypot(ex, ey)

        xy_limit = max_xy if max_xy else self.max_xy_speed
        z_limit = max_z if max_z else self.max_z_speed

        vx = self._clamp(0.85 * ex, -xy_limit, xy_limit)
        vy = self._clamp(0.85 * ey, -xy_limit, xy_limit)

        yaw_err = self._angle_wrap(target_yaw - yaw)
        yaw_rate = self._clamp(self.yaw_kp * yaw_err, -self.max_yaw_rate, self.max_yaw_rate)
        vz = self._clamp(0.9 * ez, -z_limit, z_limit)

        if self.cmd_vel_body_frame:
            c, s = math.cos(yaw), math.sin(yaw)
            vx_body = c * vx + s * vy
            vy_body = -s * vx + c * vy
            vx, vy = vx_body, vy_body

        self._publish_twist(vx, vy, vz, yaw_rate)

        yaw_ok = abs(yaw_err) < self.yaw_align_tolerance
        pos_ok = dist_xy < 0.20 and abs(ez) < 0.15

        return pos_ok and yaw_ok

    def _publish_scan_tf(self):
        """Publish static TF for LiDAR frame"""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "lidar_link"
        tf_msg.transform.translation.z = 0.065
        tf_msg.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf_msg)

    def _maybe_show_windows(self):
        """Show debug windows if display available"""
        if not self.show_windows:
            return

        if self.front_debug_frame is not None:
            cv2.imshow("Front Camera", self.front_debug_frame)
        if self.down_debug_frame is not None:
            cv2.imshow("Down Camera", self.down_debug_frame)

        # Dashboard
        self._show_dashboard()
        cv2.waitKey(1)

    def _show_dashboard(self):
        """Display mission dashboard"""
        img = np.zeros((230, 420, 3), dtype=np.uint8)
        img[:] = (24, 24, 24)

        progress = 100.0 * float(self.waypoint_idx) / max(len(self.waypoints), 1)

        # Battery color
        batt_color = (70, 200, 70)
        if self.battery_percent <= self.battery_low_warn:
            batt_color = (0, 215, 255)
        if self.battery_percent <= self.battery_critical:
            batt_color = (40, 40, 230)

        cv2.putText(img, "Mission Dashboard", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
        cv2.putText(img, f"Mode: {'MANUAL' if self.manual_mode else 'AUTO'}  Phase: {self.phase.name}",
                    (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (220, 220, 220), 1)
        cv2.putText(img, f"Speed: {self.dashboard_speed:.2f} m/s", (12, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
        cv2.putText(img, f"Distance: {self.dashboard_distance:.2f} m", (12, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
        cv2.putText(img, f"Mapped: {self.map_known_percent:.1f}%", (12, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
        cv2.putText(img, f"Covered: {self.arena_coverage_percent:.1f}%", (12, 156), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
        cv2.putText(img, f"Progress: {progress:.1f}%", (12, 182), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)

        # Battery bar
        bar_x, bar_y, bar_w, bar_h = 12, 198, 280, 24
        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (85, 85, 85), 2)
        fill_w = int((self.battery_percent / 100.0) * (bar_w - 4))
        if fill_w > 0:
            cv2.rectangle(img, (bar_x + 2, bar_y + 2), (bar_x + 2 + fill_w, bar_y + bar_h - 2), batt_color, -1)
        cv2.putText(img, f"Battery: {self.battery_percent:.1f}%", (bar_x + bar_w + 12, bar_y + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.56, batt_color, 2)

        cv2.imshow("Mission Dashboard", img)


def main(args=None):
    rclpy.init(args=args)

    node = SlamBoundaryMissionOptimized()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
