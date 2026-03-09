#!/usr/bin/env python3
"""
Optimized SLAM Boundary Mapping Mission
========================================
This is an optimized version of the SLAM mapping mission with:
- Configurable parameters via YAML
- Improved error handling and logging
- Better state machine structure
- Hardware-ready abstractions
- Performance optimizations

Author: Drone Project Team
Version: 2.0.0
"""

import math
import os
import threading
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple, Dict, Any

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
    """Mission state machine phases."""
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
    FAILSAFE_RETURN_HOME = auto()
    FAILSAFE_HOVER = auto()
    FAILSAFE_ALIGN = auto()
    FAILSAFE_DESCEND = auto()
    FAILSAFE_LAND = auto()
    # Emergency phases
    EMERGENCY_LAND = auto()
    COMM_LOSS_RTL = auto()


@dataclass
class Position:
    """3D position representation."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def distance_to(self, other: 'Position') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)
    
    def distance_3d(self, other: 'Position') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)
    
    @property
    def xy_distance(self) -> float:
        return math.hypot(self.x, self.y)


@dataclass
class Velocity:
    """3D velocity representation."""
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    
    @property
    def speed(self) -> float:
        return math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)


@dataclass
class SensorStatus:
    """Track sensor availability and freshness."""
    has_odom: bool = False
    has_scan: bool = False
    has_terrain_scan: bool = False
    has_front_camera: bool = False
    has_down_camera: bool = False
    has_motor_cmd: bool = False
    has_tof: bool = False
    
    odom_time: float = -1.0
    scan_time: float = -1.0
    front_camera_time: float = -1.0
    down_camera_time: float = -1.0
    tof_time: float = -1.0
    
    @property
    def all_required_sensors(self) -> bool:
        return all([self.has_odom, self.has_scan, self.has_front_camera, self.has_down_camera])


@dataclass
class BoundaryState:
    """Yellow boundary detection state."""
    front_yellow_ratio: float = 0.0
    down_yellow_ratio: float = 0.0
    front_seen_time: float = -1.0
    down_seen_time: float = -1.0
    repulse_body_x: float = 0.0
    repulse_body_y: float = 0.0
    repulse_strength: float = 0.0
    avoid_until: float = -1.0
    hover_until: float = -1.0
    avoid_start: float = -1.0


@dataclass
class BatteryState:
    """Battery estimation state."""
    percent: float = 100.0
    motor_load: float = 0.0
    last_update: float = 0.0


class ConfigManager:
    """Manages mission configuration parameters."""
    
    DEFAULT_CONFIG = {
        'mission': {
            'target_altitude': 4.0,
            'control_rate': 20.0,
            'base_x': 0.0,
            'base_y': 0.0,
            'landing_z': 0.20,
            'mid_altitude': 2.0,
            'home_hover_duration': 2.0,
            'mid_alt_hover_duration': 2.0,
        },
        'velocity_limits': {
            'max_xy_speed': 1.45,
            'max_z_speed': 0.6,
            'max_yaw_rate': 0.9,
        },
        'mapping': {
            'arena_half_extent': 8.0,
            'edge_margin': 1.2,
            'lane_spacing': 0.9,
            'boundary_trigger_extent': 7.2,
            'speed_boost': 1.65,
            'precision_radius': 1.4,
            'precision_min_speed': 0.55,
            'waypoint_timeout': 22.0,
            'coverage_resolution': 0.50,
        },
        'boundary_detection': {
            'front_yellow_threshold': 0.010,
            'down_yellow_threshold': 0.010,
            'yellow_hold_duration': 0.5,
            'avoid_duration': 1.2,
            'repulse_max_speed': 0.65,
            'repulse_full_ratio': 0.10,
            'guard_enter_strength': 0.08,
            'guard_exit_strength': 0.03,
            'boundary_hover_duration': 0.30,
            'retreat_base_speed': 0.35,
            'retreat_gain': 0.35,
            'avoid_max_duration': 6.0,
        },
        'obstacle_avoidance': {
            'enable': True,
            'detection_distance': 2.2,
            'repulse_gain': 0.85,
        },
        'battery': {
            'initial_percent': 100.0,
            'low_warn_percent': 25.0,
            'critical_percent': 12.0,
            'idle_drain_rate': 0.0025,
            'full_drain_rate': 0.0600,
        },
        'heading_control': {
            'yaw_kp': 1.4,
            'align_tolerance_deg': 6.0,
            'hover_before_move_duration': 0.15,
            'align_timeout': 1.8,
        },
        'safety': {
            'max_distance_from_base': 15.0,
            'max_altitude': 10.0,
            'min_altitude': 0.3,
            'command_timeout': 1.0,
        },
    }
    
    def __init__(self, config_path: Optional[str] = None):
        self.config = self._deep_copy_dict(self.DEFAULT_CONFIG)
        if config_path and os.path.exists(config_path):
            self._load_yaml(config_path)
    
    def _deep_copy_dict(self, d: Dict) -> Dict:
        return {k: self._deep_copy_dict(v) if isinstance(v, dict) else v for k, v in d.items()}
    
    def _load_yaml(self, path: str):
        """Load configuration from YAML file."""
        try:
            import yaml
            with open(path, 'r') as f:
                user_config = yaml.safe_load(f)
            self._merge_config(user_config)
        except Exception as e:
            print(f"[WARN] Failed to load config from {path}: {e}")
    
    def _merge_config(self, user_config: Dict, base: Optional[Dict] = None):
        """Recursively merge user config into defaults."""
        if base is None:
            base = self.config
        for key, value in user_config.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._merge_config(value, base[key])
            else:
                base[key] = value
    
    def get(self, *keys, default=None):
        """Get configuration value by path."""
        value = self.config
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value


class SlamBoundaryMissionOptimized(Node):
    """
    Optimized SLAM boundary mapping mission node.
    
    Improvements over original:
    - Structured state machine with clear phase transitions
    - Configurable parameters via ConfigManager
    - Better separation of concerns
    - Comprehensive error handling
    - Performance optimizations (vectorized operations)
    - Hardware-ready abstractions
    """
    
    def __init__(self, config_path: Optional[str] = None):
        super().__init__("slam_boundary_mapping_mission")
        
        # Configuration
        self.config = ConfigManager(config_path)
        
        # Initialize state
        self._init_state()
        
        # Setup ROS interfaces
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()
        
        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # CV bridge
        self.bridge = CvBridge()
        
        # Display windows
        self.show_windows = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        
        # Log startup info
        self._log_startup_info()
    
    def _init_state(self):
        """Initialize all state variables."""
        # Position and velocity
        self.position = Position()
        self.velocity = Velocity()
        self.yaw = 0.0
        self.odom_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        
        # Sensor status
        self.sensors = SensorStatus()
        
        # Mission state
        self.phase = MissionPhase.WAIT_SENSORS
        self.prev_phase = None
        self.phase_start_time = self._now()
        self.reached_since = None
        self.heading_aligned_since = None
        
        # Waypoints
        self.waypoints = self._build_waypoints()
        self.waypoint_idx = 0
        self.waypoint_start_time = None
        
        # Boundary detection
        self.boundary = BoundaryState()
        
        # Battery
        self.battery = BatteryState(percent=self.config.get('battery', 'initial_percent', default=100.0))
        
        # Coverage tracking
        self.coverage_grid = self._init_coverage_grid()
        self.coverage_percent = 0.0
        self.map_known_percent = 0.0
        
        # Manual control
        self.manual_mode = False
        self.manual_velocity = Velocity()
        self.manual_cmd_time = -1.0
        
        # Failsafe
        self.failsafe_triggered = False
        self.last_motor_warn_time = -1.0
        
        # Obstacle avoidance
        self.last_scan_ranges = None
        self.last_scan_angles = None
        
        # Command rate limiting
        self.cmd_velocity = Velocity()
        self.last_cmd_time = self._now()
        
        # Dashboard
        self.dashboard_speed = 0.0
        self.dashboard_distance = 0.0
        self.last_dashboard_log = -1.0
        
        # Debug images
        self.front_debug = None
        self.down_debug = None
    
    def _setup_publishers(self):
        """Setup all ROS publishers."""
        self.pub_enable = self.create_publisher(Bool, "/drone_enable", 10)
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_motor_cmd = self.create_publisher(Actuators, "/motor_speed_cmd_out", 10)
        self.pub_terrain_cloud = self.create_publisher(
            PointCloud2, 
            self.config.get('terrain_mapping', 'cloud_topic', default='/terrain_point_cloud'),
            10
        )
    
    def _setup_subscribers(self):
        """Setup all ROS subscribers with appropriate QoS."""
        sensor_qos = qos_profile_sensor_data
        
        # Use reentrant callback group for parallel processing
        self.callback_group = ReentrantCallbackGroup()
        
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self._odom_callback, sensor_qos
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, sensor_qos
        )
        self.sub_terrain = self.create_subscription(
            LaserScan, "/terrain_scan", self._terrain_callback, sensor_qos
        )
        self.sub_front = self.create_subscription(
            Image, "/front_camera/image_raw", self._front_camera_callback, sensor_qos
        )
        self.sub_down = self.create_subscription(
            Image, "/down_camera/image_raw", self._down_camera_callback, sensor_qos
        )
        self.sub_motor_cmd = self.create_subscription(
            Actuators, "/motor_speed_cmd_in", self._motor_cmd_callback, sensor_qos
        )
        self.sub_manual_mode = self.create_subscription(
            Bool, "/manual_mode", self._manual_mode_callback, 10
        )
        self.sub_manual_cmd = self.create_subscription(
            Twist, "/manual_cmd_vel", self._manual_cmd_callback, 10
        )
        self.sub_manual_rtb = self.create_subscription(
            Bool, "/manual_rtb", self._manual_rtb_callback, 10
        )
        self.sub_tof = self.create_subscription(
            LaserScan, "/tof_scan", self._tof_callback, sensor_qos
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, 10
        )
    
    def _setup_timers(self):
        """Setup control loop timer."""
        control_rate = self.config.get('mission', 'control_rate', default=20.0)
        self.control_timer = self.create_timer(1.0 / control_rate, self._control_tick)
    
    def _now(self) -> float:
        """Get current time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9
    
    @staticmethod
    def _clamp(val: float, low: float, high: float) -> float:
        return max(low, min(high, val))
    
    @staticmethod
    def _angle_wrap(rad: float) -> float:
        return math.atan2(math.sin(rad), math.cos(rad))
    
    # ==================== Waypoint Generation ====================
    
    def _build_waypoints(self) -> List[Tuple[float, float, float]]:
        """Build lane-by-lane sweep waypoints."""
        cfg = self.config.config['mapping']
        z = self.config.get('mission', 'target_altitude', default=4.0)
        
        half_extent = cfg['arena_half_extent']
        margin = cfg['edge_margin']
        spacing = cfg['lane_spacing']
        
        x_min = -half_extent + margin
        x_max = half_extent - margin
        y_min = -half_extent + margin
        y_max = half_extent - margin
        
        span_y = max(0.1, y_max - y_min)
        lane_count = max(1, int(math.floor(span_y / spacing)) + 1)
        lane_step = 0.0 if lane_count == 1 else span_y / float(lane_count - 1)
        
        waypoints = []
        for lane_idx in range(lane_count):
            y_lane = y_min + lane_idx * lane_step
            
            # Alternating direction for efficient coverage
            if lane_idx % 2 == 0:
                x_start, x_end = x_min, x_max
            else:
                x_start, x_end = x_max, x_min
            
            start_wp = (x_start, y_lane, z)
            end_wp = (x_end, y_lane, z)
            
            # Avoid duplicates
            if not waypoints or any(abs(start_wp[i] - waypoints[-1][i]) > 1e-6 for i in range(3)):
                waypoints.append(start_wp)
            if any(abs(end_wp[i] - waypoints[-1][i]) > 1e-6 for i in range(3)):
                waypoints.append(end_wp)
        
        return waypoints
    
    def _init_coverage_grid(self) -> np.ndarray:
        """Initialize coverage tracking grid."""
        extent = 2.0 * self.config.get('mapping', 'arena_half_extent', default=8.0)
        resolution = max(0.1, self.config.get('mapping', 'coverage_resolution', default=0.5))
        cells = max(1, int(math.ceil(extent / resolution)))
        return np.zeros((cells, cells), dtype=bool)
    
    # ==================== Callbacks ====================
    
    def _odom_callback(self, msg: Odometry):
        """Process odometry message."""
        self.sensors.has_odom = True
        self.sensors.odom_time = self._now()
        
        # Update position
        self.position.x = msg.pose.pose.position.x
        self.position.y = msg.pose.pose.position.y
        self.position.z = msg.pose.pose.position.z
        
        # Update velocity
        self.velocity.vx = msg.twist.twist.linear.x
        self.velocity.vy = msg.twist.twist.linear.y
        self.velocity.vz = msg.twist.twist.linear.z
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.odom_quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        # Publish TF
        self._publish_odom_tf(msg)
    
    def _publish_odom_tf(self, msg: Odometry):
        """Publish odometry to base_link transform."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp if msg.header.stamp.sec > 0 else self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)
    
    def _scan_callback(self, msg: LaserScan):
        """Process LiDAR scan for SLAM and obstacle avoidance."""
        self.sensors.has_scan = True
        self.sensors.scan_time = self._now()
        
        # Store for obstacle avoidance
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        
        self.last_scan_ranges = ranges[valid]
        angles = msg.angle_min + np.arange(len(msg.ranges), dtype=np.float64) * msg.angle_increment
        self.last_scan_angles = angles[valid]
    
    def _terrain_callback(self, msg: LaserScan):
        """Process terrain LiDAR scan and publish point cloud."""
        self.sensors.has_terrain_scan = True
        
        if not self.sensors.has_odom or not msg.ranges:
            return
        
        # Generate terrain point cloud
        points = self._process_terrain_scan(msg)
        if points is not None:
            self._publish_point_cloud(msg.header.stamp, points)
    
    def _process_terrain_scan(self, msg: LaserScan) -> Optional[np.ndarray]:
        """Convert terrain scan to 3D points in odom frame."""
        stride = max(1, int(self.config.get('terrain_mapping', 'point_stride', default=2)))
        ranges = np.asarray(msg.ranges, dtype=np.float64)[::stride]
        ray_idx = np.arange(0, len(msg.ranges), stride, dtype=np.float64)
        angles = msg.angle_min + ray_idx * msg.angle_increment
        
        # Filter valid ranges
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(valid):
            return None
        
        ranges = ranges[valid]
        angles = angles[valid]
        
        # Sensor frame points
        points_sensor = np.vstack([
            ranges * np.cos(angles),
            ranges * np.sin(angles),
            np.zeros_like(ranges)
        ])
        
        # Transform to base_link (pitched LiDAR)
        pitch = math.radians(self.config.get('terrain_mapping', 'lidar_pitch_deg', default=-30.0))
        rot_sensor = self._rpy_to_rotation_matrix(0, pitch, 0)
        offset = np.array(self.config.get('terrain_mapping', 'lidar_offset', default=[0, 0, 0.065]))
        
        points_base = rot_sensor @ points_sensor + offset.reshape(3, 1)
        
        # Transform to odom frame
        rot_odom = self._quat_to_rotation_matrix(self.odom_quat)
        base_pos = np.array([self.position.x, self.position.y, self.position.z]).reshape(3, 1)
        points_odom = rot_odom @ points_base + base_pos
        
        return points_odom.T.astype(np.float32)
    
    @staticmethod
    def _rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert roll-pitch-yaw to rotation matrix."""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ], dtype=np.float64)
    
    @staticmethod
    def _quat_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
        """Convert quaternion to rotation matrix."""
        qx, qy, qz, qw = quat
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-9:
            return np.eye(3, dtype=np.float64)
        qx /= norm; qy /= norm; qz /= norm; qw /= norm
        
        return np.array([
            [1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw), 2.0*(qx*qz + qy*qw)],
            [2.0*(qx*qy + qz*qw), 1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw)],
            [2.0*(qx*qz - qy*qw), 2.0*(qy*qz + qx*qw), 1.0 - 2.0*(qx*qx + qy*qy)]
        ], dtype=np.float64)
    
    def _publish_point_cloud(self, stamp, points: np.ndarray):
        """Publish point cloud message."""
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "odom"
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = points.tobytes()
        self.pub_terrain_cloud.publish(cloud)
    
    def _front_camera_callback(self, msg: Image):
        """Process front camera image for boundary detection."""
        self.sensors.has_front_camera = True
        self.sensors.front_camera_time = self._now()
        
        ratio, vis = self._detect_yellow(msg)
        self.boundary.front_yellow_ratio = ratio
        
        if ratio > self.config.get('boundary_detection', 'front_yellow_threshold', default=0.01):
            self.boundary.front_seen_time = self._now()
        
        self.front_debug = vis
    
    def _down_camera_callback(self, msg: Image):
        """Process downward camera for boundary detection with repulsion vector."""
        self.sensors.has_down_camera = True
        self.sensors.down_camera_time = self._now()
        
        ratio, vis, rep_x, rep_y, strength = self._detect_yellow_with_repulsion(msg)
        self.boundary.down_yellow_ratio = ratio
        
        if ratio > self.config.get('boundary_detection', 'down_yellow_threshold', default=0.01):
            self.boundary.down_seen_time = self._now()
        
        self.boundary.repulse_body_x = rep_x
        self.boundary.repulse_body_y = rep_y
        self.boundary.repulse_strength = strength
        self.down_debug = vis
    
    def _detect_yellow(self, msg: Image) -> Tuple[float, Optional[np.ndarray]]:
        """Detect yellow in image and return ratio and visualization."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Image decode failed: {e}")
            return 0.0, None
        
        mask = self._yellow_mask(frame)
        ratio = float(np.count_nonzero(mask)) / float(mask.size)
        
        vis = frame.copy()
        vis[mask > 0] = (0, 255, 255)
        cv2.putText(vis, f"yellow_ratio={ratio:.3f}", (20, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        
        return ratio, vis
    
    def _detect_yellow_with_repulsion(self, msg: Image) -> Tuple[float, Optional[np.ndarray], float, float, float]:
        """Detect yellow and compute repulsion vector."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Image decode failed: {e}")
            return 0.0, None, 0.0, 0.0, 0.0
        
        mask = self._yellow_mask(frame)
        ratio = float(np.count_nonzero(mask)) / float(mask.size)
        
        h, w = mask.shape
        cx0, cy0 = 0.5 * (w - 1), 0.5 * (h - 1)
        
        rep_x, rep_y, strength = 0.0, 0.0, 0.0
        centroid = None
        
        # Compute centroid of yellow pixels
        m = cv2.moments(mask, binaryImage=True)
        if m["m00"] > 1e-6:
            cx = float(m["m10"] / m["m00"])
            cy = float(m["m01"] / m["m00"])
            centroid = (int(cx), int(cy))
            
            # Normalized offset from center
            off_x = (cx - cx0) / max(1.0, 0.5 * w)
            off_y = (cy - cy0) / max(1.0, 0.5 * h)
            
            # Repulsion direction (opposite to centroid)
            rep_img_x, rep_img_y = -off_x, -off_y
            n = math.hypot(rep_img_x, rep_img_y)
            
            if n > 1e-6:
                rep_img_x /= n
                rep_img_y /= n
                
                # Convert to body frame (image top = forward, image right = body right)
                body_x = -rep_img_y
                body_y = -rep_img_x
                
                # Strength based on ratio
                threshold = self.config.get('boundary_detection', 'down_yellow_threshold', default=0.01)
                full_ratio = self.config.get('boundary_detection', 'repulse_full_ratio', default=0.10)
                max_speed = self.config.get('boundary_detection', 'repulse_max_speed', default=0.65)
                
                strength = self._clamp((ratio - threshold) / max(1e-6, full_ratio - threshold), 0.0, 1.0)
                speed = max_speed * strength
                
                rep_x = speed * body_x
                rep_y = speed * body_y
        
        # Visualization
        vis = frame.copy()
        vis[mask > 0] = (0, 255, 255)
        
        center_pt = (int(cx0), int(cy0))
        cv2.circle(vis, center_pt, 5, (255, 255, 255), -1, cv2.LINE_AA)
        
        if centroid:
            cv2.circle(vis, centroid, 6, (0, 0, 255), 2, cv2.LINE_AA)
        
        if strength > 0:
            arrow_len = int(35 + 160 * strength)
            arrow_end = (int(center_pt[0] + rep_img_x * arrow_len),
                        int(center_pt[1] + rep_img_y * arrow_len))
            cv2.arrowedLine(vis, center_pt, arrow_end, (0, 0, 255), 3, cv2.LINE_AA, tipLength=0.25)
        
        cv2.putText(vis, f"yellow={ratio:.3f} repel={strength:.2f}", (20, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        
        return ratio, vis, rep_x, rep_y, strength
    
    def _yellow_mask(self, frame: np.ndarray) -> np.ndarray:
        """Create yellow color mask using HSV thresholding."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower = np.array([
            self.config.get('boundary_detection', 'yellow_h_min', default=18),
            self.config.get('boundary_detection', 'yellow_s_min', default=80),
            self.config.get('boundary_detection', 'yellow_v_min', default=80)
        ], dtype=np.uint8)
        
        upper = np.array([
            self.config.get('boundary_detection', 'yellow_h_max', default=38),
            self.config.get('boundary_detection', 'yellow_s_max', default=255),
            self.config.get('boundary_detection', 'yellow_v_max', default=255)
        ], dtype=np.uint8)
        
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    def _motor_cmd_callback(self, msg: Actuators):
        """Process motor command for ESC relay and load estimation."""
        self.sensors.has_motor_cmd = True
        
        out = Actuators()
        out.header = msg.header
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.normalized = list(msg.normalized)
        
        # Apply failsafe motor cut if triggered
        if self.failsafe_triggered:
            failed_idx = self.config.get('failsafe', 'failed_motor_index', default=3)
            if len(out.velocity) > failed_idx:
                out.velocity[failed_idx] = 0.0
            if len(out.normalized) > failed_idx:
                out.normalized[failed_idx] = 0.0
        
        # Estimate motor load
        if out.normalized:
            self.battery.motor_load = self._clamp(float(np.mean(np.abs(np.asarray(out.normalized)))), 0.0, 1.0)
        elif out.velocity:
            vel = np.asarray(out.velocity, dtype=np.float64)
            finite = vel[np.isfinite(vel)]
            if finite.size > 0:
                self.battery.motor_load = self._clamp(
                    float(np.mean(np.abs(finite)) / max(1.0, np.max(np.abs(finite)))),
                    0.0, 1.0
                )
        
        if rclpy.ok():
            try:
                self.pub_motor_cmd.publish(out)
            except Exception:
                pass
    
    def _tof_callback(self, msg: LaserScan):
        """Process Time-of-Flight sensor for precise altitude."""
        if not msg.ranges:
            self.sensors.has_tof = False
            return
        
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        finite = ranges[np.isfinite(ranges)]
        
        if finite.size == 0:
            self.sensors.has_tof = False
            return
        
        valid = finite[(finite >= msg.range_min) & (finite <= msg.range_max)]
        if valid.size == 0:
            self.sensors.has_tof = False
            return
        
        self.sensors.tof_time = self._now()
        self.sensors.tof_range = float(np.min(valid))
        self.sensors.has_tof = True
    
    def _map_callback(self, msg: OccupancyGrid):
        """Process SLAM map for coverage estimation."""
        if not msg.data:
            return
        
        data = np.asarray(msg.data, dtype=np.int16)
        total = data.size
        if total == 0:
            return
        
        known = int(np.count_nonzero(data >= 0))
        self.map_known_percent = 100.0 * float(known) / float(total)
    
    def _manual_mode_callback(self, msg: Bool):
        """Handle manual mode toggle."""
        prev = self.manual_mode
        self.manual_mode = bool(msg.data)
        
        if self.manual_mode != prev:
            mode_str = "MANUAL" if self.manual_mode else "AUTONOMOUS"
            self.get_logger().warn(f"Control mode -> {mode_str}")
            self._reset_waypoint_state()
        
        if not self.manual_mode:
            self.manual_velocity = Velocity()
    
    def _manual_cmd_callback(self, msg: Twist):
        """Handle manual velocity commands."""
        self.manual_cmd_time = self._now()
        
        max_xy = self.config.get('manual_control', 'max_xy_speed', default=1.4)
        max_z = self.config.get('manual_control', 'max_z_speed', default=0.7)
        max_yaw = self.config.get('manual_control', 'max_yaw_rate', default=1.2)
        
        self.manual_velocity.vx = self._clamp(msg.linear.x, -max_xy, max_xy)
        self.manual_velocity.vy = self._clamp(msg.linear.y, -max_xy, max_xy)
        self.manual_velocity.vz = self._clamp(msg.linear.z, -max_z, max_z)
        self.manual_velocity.yaw_rate = self._clamp(msg.angular.z, -max_yaw, max_yaw)
    
    def _manual_rtb_callback(self, msg: Bool):
        """Handle manual return-to-base request."""
        if not msg.data:
            return
        
        self.get_logger().warn("Manual RTB requested: switching to return and landing sequence.")
        self.manual_mode = False
        self.manual_velocity = Velocity()
        self._set_phase(MissionPhase.RETURN_HOME_AT_4M)
    
    # ==================== Control Logic ====================
    
    def _control_tick(self):
        """Main control loop callback."""
        now = self._now()
        
        # Update estimates
        self._update_battery(now)
        self._update_coverage(now)
        
        # Show debug windows
        self._show_debug_windows()
        
        # Publish static TF
        self._publish_scan_tf()
        
        # Check for communication loss
        if self._check_comm_loss(now):
            return
        
        # Manual mode handling
        if self.manual_mode:
            self._run_manual_control(now)
            return
        
        # Check sensors
        if not self.sensors.all_required_sensors:
            self._publish_enable(False)
            self._publish_velocity(0, 0, 0, 0)
            return
        
        # Mission complete
        if self.phase == MissionPhase.DONE:
            self._publish_enable(False)
            self._publish_velocity(0, 0, 0, 0)
            return
        
        # Enable controller
        self._publish_enable(True)
        
        # Check failsafe trigger
        self._check_failsafe_trigger()
        
        # Run current phase
        self._run_phase(now)
    
    def _run_phase(self, now: float):
        """Execute current mission phase logic."""
        phase_handlers = {
            MissionPhase.WAIT_SENSORS: self._phase_wait_sensors,
            MissionPhase.TAKEOFF: self._phase_takeoff,
            MissionPhase.MAPPING: self._phase_mapping,
            MissionPhase.RETURN_HOME_AT_4M: self._phase_return_home,
            MissionPhase.HOVER_HOME_AT_4M: self._phase_hover_home,
            MissionPhase.ALIGN_YAW_AT_4M: self._phase_align_yaw,
            MissionPhase.DESCEND_TO_2M: self._phase_descend,
            MissionPhase.HOLD_AT_2M: self._phase_hold,
            MissionPhase.ALIGN_YAW_AT_2M: self._phase_align_yaw_2m,
            MissionPhase.LAND: self._phase_land,
            MissionPhase.FAILSAFE_RETURN_HOME: self._phase_failsafe_return,
            MissionPhase.FAILSAFE_LAND: self._phase_failsafe_land,
            MissionPhase.EMERGENCY_LAND: self._phase_emergency_land,
        }
        
        handler = phase_handlers.get(self.phase)
        if handler:
            handler(now)
    
    def _phase_wait_sensors(self, now: float):
        """Wait for all required sensors."""
        self._set_phase(MissionPhase.TAKEOFF)
    
    def _phase_takeoff(self, now: float):
        """Takeoff to target altitude."""
        target_z = self.config.get('mission', 'target_altitude', default=4.0)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        reached = self._goto_position(base_x, base_y, target_z)
        
        if reached:
            if self.reached_since is None:
                self.reached_since = now
            elif now - self.reached_since > 1.5:
                self._set_phase(MissionPhase.MAPPING)
                self.get_logger().info("Takeoff complete. Starting mapping sweep.")
        else:
            self.reached_since = None
    
    def _phase_mapping(self, now: float):
        """Execute mapping sweep with boundary avoidance."""
        # Check for stuck condition
        if self._is_mapping_stuck(now):
            self.get_logger().warn(f"Waypoint {self.waypoint_idx + 1} appears stuck; advancing.")
            self._advance_waypoint()
            return
        
        # Handle boundary avoidance
        if now < self.boundary.avoid_until:
            if self._should_continue_boundary_avoidance(now):
                self._run_boundary_avoidance(now)
                return
            else:
                self._reset_boundary_avoidance()
        
        # Check if all waypoints complete
        if self.waypoint_idx >= len(self.waypoints):
            self._set_phase(MissionPhase.RETURN_HOME_AT_4M)
            return
        
        # Check waypoint timeout
        if self.waypoint_start_time is None:
            self.waypoint_start_time = now
        elif now - self.waypoint_start_time > self.config.get('mapping', 'waypoint_timeout', default=22.0):
            self.get_logger().warn(f"Waypoint {self.waypoint_idx + 1} timeout; advancing.")
            self._advance_waypoint()
            return
        
        # Check boundary trigger
        if self._should_trigger_boundary_avoidance(now):
            self._start_boundary_avoidance(now)
            return
        
        # Navigate to waypoint
        tx, ty, tz = self.waypoints[self.waypoint_idx]
        reached = self._goto_position(tx, ty, tz)
        
        if reached:
            if self.reached_since is None:
                self.reached_since = now
            elif now - self.reached_since > 0.15:
                self.get_logger().info(f"Waypoint {self.waypoint_idx + 1}/{len(self.waypoints)} complete")
                self._advance_waypoint()
        else:
            self.reached_since = None
    
    def _phase_return_home(self, now: float):
        """Return to base at mapping altitude."""
        target_z = self.config.get('mission', 'target_altitude', default=4.0)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        reached = self._goto_position(base_x, base_y, target_z)
        
        if reached:
            if self.reached_since is None:
                self.reached_since = now
            elif now - self.reached_since > 0.8:
                self._set_phase(MissionPhase.HOVER_HOME_AT_4M)
        else:
            self.reached_since = None
    
    def _phase_hover_home(self, now: float):
        """Hover over base before descent."""
        target_z = self.config.get('mission', 'target_altitude', default=4.0)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        self._goto_position(base_x, base_y, target_z)
        
        hover_duration = self.config.get('mission', 'home_hover_duration', default=2.0)
        if now - self.phase_start_time > hover_duration:
            self._set_phase(MissionPhase.ALIGN_YAW_AT_4M)
    
    def _phase_align_yaw(self, now: float):
        """Align yaw for landing approach."""
        target_z = self.config.get('mission', 'target_altitude', default=4.0)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        pos_ok, yaw_ok = self._goto_with_yaw(base_x, base_y, target_z, 0.0)
        
        if pos_ok and yaw_ok:
            if self.reached_since is None:
                self.reached_since = now
            elif now - self.reached_since > 0.6:
                self._set_phase(MissionPhase.DESCEND_TO_2M)
        else:
            self.reached_since = None
    
    def _phase_descend(self, now: float):
        """Descend to mid altitude."""
        mid_z = self.config.get('mission', 'mid_altitude', default=2.0)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        reached = self._goto_position(base_x, base_y, mid_z, max_z_speed=0.16)
        
        if reached:
            if self.reached_since is None:
                self.reached_since = now
            elif now - self.reached_since > 0.8:
                self._set_phase(MissionPhase.HOLD_AT_2M)
        else:
            self.reached_since = None
    
    def _phase_hold(self, now: float):
        """Hold at mid altitude."""
        mid_z = self.config.get('mission', 'mid_altitude', default=2.0)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        self._goto_position(base_x, base_y, mid_z, max_z_speed=0.16)
        
        hover_duration = self.config.get('mission', 'mid_alt_hover_duration', default=2.0)
        if now - self.phase_start_time > hover_duration:
            self._set_phase(MissionPhase.ALIGN_YAW_AT_2M)
    
    def _phase_align_yaw_2m(self, now: float):
        """Final yaw alignment before landing."""
        mid_z = self.config.get('mission', 'mid_altitude', default=2.0)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        pos_ok, yaw_ok = self._goto_with_yaw(base_x, base_y, mid_z, 0.0, max_xy_speed=0.35)
        
        if pos_ok and yaw_ok:
            if self.reached_since is None:
                self.reached_since = now
            elif now - self.reached_since > 0.6:
                self._set_phase(MissionPhase.LAND)
        else:
            self.reached_since = None
    
    def _phase_land(self, now: float):
        """Final landing phase."""
        landing_z = self.config.get('mission', 'landing_z', default=0.20)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        pos_ok, yaw_ok = self._goto_with_yaw(base_x, base_y, landing_z, 0.0, max_xy_speed=0.25, max_z_speed=0.12)
        
        if pos_ok and yaw_ok:
            if self.reached_since is None:
                self.reached_since = now
            elif now - self.reached_since > 1.0:
                self.get_logger().info("Landing complete.")
                self._set_phase(MissionPhase.DONE)
        else:
            self.reached_since = None
    
    def _phase_failsafe_return(self, now: float):
        """Failsafe return with reduced speed."""
        target_z = self.config.get('mission', 'target_altitude', default=4.0)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        reached = self._goto_position(base_x, base_y, target_z, max_xy_speed=0.45, max_z_speed=0.20)
        
        if reached:
            if self.reached_since is None:
                self.reached_since = now
            elif now - self.reached_since > 0.8:
                self._set_phase(MissionPhase.FAILSAFE_LAND)
        else:
            self.reached_since = None
    
    def _phase_failsafe_land(self, now: float):
        """Failsafe landing with reduced speed."""
        landing_z = self.config.get('mission', 'landing_z', default=0.20)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        pos_ok, _ = self._goto_with_yaw(base_x, base_y, landing_z, 0.0, max_xy_speed=0.25, max_z_speed=0.08)
        
        if pos_ok:
            if self.reached_since is None:
                self.reached_since = now
            elif now - self.reached_since > 1.5:
                self.get_logger().info("Failsafe landing complete.")
                self._set_phase(MissionPhase.DONE)
        else:
            self.reached_since = None
    
    def _phase_emergency_land(self, now: float):
        """Emergency landing - immediate descent."""
        vz = -0.3  # Faster descent
        self._publish_velocity(0, 0, vz, 0)
        
        if self.position.z < 0.15:
            self._publish_velocity(0, 0, 0, 0)
            self._publish_enable(False)
            self._set_phase(MissionPhase.DONE)
    
    # ==================== Navigation Helpers ====================
    
    def _goto_position(self, x: float, y: float, z: float, 
                       max_xy_speed: Optional[float] = None,
                       max_z_speed: Optional[float] = None) -> bool:
        """Navigate to position. Returns True when reached."""
        now = self._now()
        
        # Check battery for auto-RTL
        if self.battery.percent < self.config.get('battery', 'low_warn_percent', default=25.0):
            if self.phase == MissionPhase.MAPPING:
                self.get_logger().warn(f"Low battery ({self.battery.percent:.1f}%). Initiating RTL.")
                self._set_phase(MissionPhase.RETURN_HOME_AT_4M)
                return False
        
        # Calculate position error
        ex = x - self.position.x
        ey = y - self.position.y
        ez = z - self.position.z
        
        dist_xy = math.hypot(ex, ey)
        
        # Speed limits
        xy_limit = max_xy_speed or self.config.get('velocity_limits', 'max_xy_speed', default=1.45)
        z_limit = max_z_speed or self.config.get('velocity_limits', 'max_z_speed', default=0.6)
        
        # Apply mapping speed profile
        if self.phase == MissionPhase.MAPPING and max_xy_speed is None:
            boost = self.config.get('mapping', 'speed_boost', default=1.65)
            precision_r = self.config.get('mapping', 'precision_radius', default=1.4)
            precision_min = self.config.get('mapping', 'precision_min_speed', default=0.55)
            
            xy_limit = boost
            if dist_xy < precision_r:
                alpha = self._clamp(dist_xy / max(precision_r, 1e-6), 0.0, 1.0)
                xy_limit = precision_min + (boost - precision_min) * alpha
        
        # Compute base velocity
        vx = self._clamp(0.85 * ex, -xy_limit, xy_limit)
        vy = self._clamp(0.85 * ey, -xy_limit, xy_limit)
        
        # Add repulsion vectors
        if self.phase == MissionPhase.MAPPING:
            rep_vx, rep_vy = self._get_repulsion_vector()
            vx += rep_vx
            vy += rep_vy
        
        # Limit final velocity
        speed = math.hypot(vx, vy)
        if speed > xy_limit:
            scale = xy_limit / max(speed, 1e-6)
            vx *= scale
            vy *= scale
        
        # Compute vertical velocity
        vz = self._compute_vertical_velocity(z, z_limit, now)
        
        # Publish command
        self._publish_velocity(vx, vy, vz, 0)
        
        # Check arrival
        z_err = self._get_z_error(z, now)
        return dist_xy < 0.25 and abs(z_err) < 0.20
    
    def _goto_with_yaw(self, x: float, y: float, z: float, target_yaw: float,
                       max_xy_speed: Optional[float] = None,
                       max_z_speed: Optional[float] = None) -> Tuple[bool, bool]:
        """Navigate to position with yaw alignment. Returns (pos_ok, yaw_ok)."""
        now = self._now()
        
        ex = x - self.position.x
        ey = y - self.position.y
        dist_xy = math.hypot(ex, ey)
        
        xy_limit = max_xy_speed or self.config.get('velocity_limits', 'max_xy_speed', default=1.45)
        z_limit = max_z_speed or self.config.get('velocity_limits', 'max_z_speed', default=0.6)
        
        # Compute velocity
        vx = self._clamp(0.85 * ex, -xy_limit, xy_limit)
        vy = self._clamp(0.85 * ey, -xy_limit, xy_limit)
        
        # Compute yaw rate
        yaw_kp = self.config.get('heading_control', 'yaw_kp', default=1.4)
        max_yaw = self.config.get('velocity_limits', 'max_yaw_rate', default=0.9)
        yaw_err = self._angle_wrap(target_yaw - self.yaw)
        yaw_rate = self._clamp(yaw_kp * yaw_err, -max_yaw, max_yaw)
        
        # Compute vertical velocity
        vz = self._compute_vertical_velocity(z, z_limit, now)
        
        # Publish command
        self._publish_velocity(vx, vy, vz, yaw_rate)
        
        # Check status
        z_err = self._get_z_error(z, now)
        tol_deg = self.config.get('heading_control', 'align_tolerance_deg', default=6.0)
        
        pos_ok = dist_xy < 0.20 and abs(z_err) < 0.15
        yaw_ok = abs(yaw_err) < math.radians(tol_deg)
        
        return pos_ok, yaw_ok
    
    def _compute_vertical_velocity(self, z_ref: float, z_limit: float, now: float) -> float:
        """Compute vertical velocity with ToF blending."""
        ez = z_ref - self.position.z
        
        # Blend with ToF if valid
        if self._tof_is_valid(now):
            tof_weight = self._get_tof_weight()
            ez_tof = z_ref - self.sensors.tof_range
            ez = (1.0 - tof_weight) * ez + tof_weight * ez_tof
        
        return self._clamp(0.9 * ez, -z_limit, z_limit)
    
    def _get_z_error(self, z_ref: float, now: float) -> float:
        """Get z error with ToF blending."""
        ez = z_ref - self.position.z
        if self._tof_is_valid(now):
            tof_weight = self._get_tof_weight()
            ez_tof = z_ref - self.sensors.tof_range
            ez = (1.0 - tof_weight) * ez + tof_weight * ez_tof
        return ez
    
    def _tof_is_valid(self, now: float) -> bool:
        """Check if ToF reading is valid and fresh."""
        if not self.sensors.has_tof:
            return False
        min_range = self.config.get('altitude_control', 'tof_min_range', default=0.05)
        max_range = self.config.get('altitude_control', 'tof_max_range', default=6.0)
        timeout = self.config.get('altitude_control', 'tof_timeout', default=0.5)
        
        return (self.sensors.tof_range > min_range and 
                self.sensors.tof_range < max_range and
                (now - self.sensors.tof_time) < timeout)
    
    def _get_tof_weight(self) -> float:
        """Get ToF blend weight based on range."""
        full_weight_range = self.config.get('altitude_control', 'tof_full_weight_range', default=2.5)
        max_range = self.config.get('altitude_control', 'tof_max_range', default=6.0)
        base_weight = self.config.get('altitude_control', 'tof_blend_weight', default=0.65)
        
        if self.sensors.tof_range <= full_weight_range:
            return base_weight
        if self.sensors.tof_range >= max_range:
            return 0.0
        
        scale = 1.0 - (self.sensors.tof_range - full_weight_range) / (max_range - full_weight_range)
        return base_weight * self._clamp(scale, 0.0, 1.0)
    
    def _get_repulsion_vector(self) -> Tuple[float, float]:
        """Compute combined repulsion from boundaries and obstacles."""
        rep_vx, rep_vy = 0.0, 0.0
        
        # Boundary repulsion
        if self._near_boundary():
            bx, by = self._boundary_repulsion_world()
            rep_vx += bx
            rep_vy += by
        
        # Obstacle repulsion
        if self.config.get('obstacle_avoidance', 'enable', default=True):
            ox, oy = self._obstacle_repulsion_world()
            rep_vx += ox
            rep_vy += oy
        
        return rep_vx, rep_vy
    
    def _near_boundary(self) -> bool:
        """Check if near arena boundary."""
        extent = self.config.get('mapping', 'boundary_trigger_extent', default=7.2)
        return abs(self.position.x) >= extent or abs(self.position.y) >= extent
    
    def _boundary_repulsion_world(self) -> Tuple[float, float]:
        """Get boundary repulsion in world frame."""
        # Transform body repulsion to world frame
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vx = c * self.boundary.repulse_body_x - s * self.boundary.repulse_body_y
        vy = s * self.boundary.repulse_body_x + c * self.boundary.repulse_body_y
        return vx, vy
    
    def _obstacle_repulsion_world(self) -> Tuple[float, float]:
        """Compute obstacle repulsion in world frame."""
        if self.last_scan_ranges is None or self.last_scan_ranges.size == 0:
            return 0.0, 0.0
        
        threshold = self.config.get('obstacle_avoidance', 'detection_distance', default=2.2)
        gain = self.config.get('obstacle_avoidance', 'repulse_gain', default=0.85)
        
        # Find close obstacles
        close_mask = self.last_scan_ranges < threshold
        if not np.any(close_mask):
            return 0.0, 0.0
        
        close_ranges = self.last_scan_ranges[close_mask]
        close_angles = self.last_scan_angles[close_mask]
        
        # Inverse-square repulsion
        weights = 1.0 / np.square(close_ranges + 0.05)
        body_x = -np.sum(weights * np.cos(close_angles))
        body_y = -np.sum(weights * np.sin(close_angles))
        
        # Normalize and apply gain
        mag = math.hypot(body_x, body_y)
        if mag > 1e-6:
            body_x = (body_x / mag) * gain
            body_y = (body_y / mag) * gain
        
        # Transform to world frame
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        world_x = c * body_x - s * body_y
        world_y = s * body_x + c * body_y
        
        return world_x, world_y
    
    # ==================== Boundary Avoidance ====================
    
    def _should_trigger_boundary_avoidance(self, now: float) -> bool:
        """Check if boundary avoidance should be triggered."""
        if self.phase != MissionPhase.MAPPING or self.failsafe_triggered:
            return False
        
        near_boundary = self._near_boundary()
        enter_strength = self.config.get('boundary_detection', 'guard_enter_strength', default=0.08)
        
        recent_yellow = self._yellow_is_recent(now)
        
        return near_boundary and (self.boundary.repulse_strength >= enter_strength or recent_yellow)
    
    def _yellow_is_recent(self, now: float) -> bool:
        """Check if yellow was recently detected."""
        hold = self.config.get('boundary_detection', 'yellow_hold_duration', default=0.5)
        return ((now - self.boundary.front_seen_time) < hold or 
                (now - self.boundary.down_seen_time) < hold)
    
    def _start_boundary_avoidance(self, now: float):
        """Initialize boundary avoidance."""
        hover_duration = self.config.get('boundary_detection', 'boundary_hover_duration', default=0.30)
        avoid_duration = self.config.get('boundary_detection', 'avoid_duration', default=1.2)
        max_duration = self.config.get('boundary_detection', 'avoid_max_duration', default=6.0)
        
        self.boundary.hover_until = now + hover_duration
        if self.boundary.avoid_start < 0:
            self.boundary.avoid_start = now
        
        deadline = self.boundary.avoid_start + max_duration
        extension = avoid_duration + self.boundary.repulse_strength
        self.boundary.avoid_until = min(max(self.boundary.avoid_until, now + extension), deadline)
        
        self.get_logger().info(f"Boundary avoidance triggered. Strength: {self.boundary.repulse_strength:.2f}")
    
    def _should_continue_boundary_avoidance(self, now: float) -> bool:
        """Check if boundary avoidance should continue."""
        max_duration = self.config.get('boundary_detection', 'avoid_max_duration', default=6.0)
        
        if self.boundary.avoid_start > 0 and (now - self.boundary.avoid_start) > max_duration:
            self.get_logger().warn("Boundary avoidance max duration reached.")
            return False
        return True
    
    def _run_boundary_avoidance(self, now: float):
        """Execute boundary avoidance maneuver."""
        # Compute retreat vector (toward center)
        base_x = self.config.get('mission', 'base_x', default=0.0)
        base_y = self.config.get('mission', 'base_y', default=0.0)
        
        in_x = base_x - self.position.x
        in_y = base_y - self.position.y
        in_mag = math.hypot(in_x, in_y)
        
        if in_mag > 1e-6:
            dir_x, dir_y = in_x / in_mag, in_y / in_mag
        else:
            # Use repulsion vector if at center
            dir_x, dir_y = self._boundary_repulsion_world()
            mag = math.hypot(dir_x, dir_y)
            if mag > 1e-6:
                dir_x, dir_y = dir_x / mag, dir_y / mag
            else:
                dir_x, dir_y = 1.0, 0.0
        
        # Compute speed
        base_speed = self.config.get('boundary_detection', 'retreat_base_speed', default=0.35)
        retreat_gain = self.config.get('boundary_detection', 'retreat_gain', default=0.35)
        max_speed = self.config.get('velocity_limits', 'max_xy_speed', default=1.45)
        
        strength = max(self.boundary.repulse_strength, self.boundary.down_yellow_ratio)
        speed = self._clamp(base_speed + retreat_gain * strength, base_speed, max_speed)
        
        vx = speed * dir_x
        vy = speed * dir_y
        
        # Compute yaw to align with retreat direction
        target_yaw = math.atan2(vy, vx)
        yaw_err = self._angle_wrap(target_yaw - self.yaw)
        yaw_kp = self.config.get('heading_control', 'yaw_kp', default=1.4)
        max_yaw = self.config.get('velocity_limits', 'max_yaw_rate', default=0.9)
        yaw_rate = self._clamp(yaw_kp * yaw_err, -max_yaw, max_yaw)
        
        # Compute vertical velocity
        target_z = self.config.get('mission', 'target_altitude', default=4.0)
        vz = self._compute_vertical_velocity(target_z, 0.25, now)
        
        # Hover phase
        if now < self.boundary.hover_until:
            self._publish_velocity(0, 0, vz, yaw_rate)
            return
        
        # Check yaw alignment
        tol_deg = self.config.get('boundary_detection', 'move_yaw_tolerance_deg', default=6.0)
        if abs(yaw_err) > math.radians(tol_deg):
            self._publish_velocity(0, 0, vz, yaw_rate)
            return
        
        self._publish_velocity(vx, vy, vz, yaw_rate)
    
    def _reset_boundary_avoidance(self):
        """Reset boundary avoidance state."""
        self.boundary.avoid_until = -1.0
        self.boundary.hover_until = -1.0
        self.boundary.avoid_start = -1.0
    
    # ==================== Utility Methods ====================
    
    def _set_phase(self, phase: MissionPhase):
        """Set mission phase with logging."""
        if phase != self.phase:
            self.prev_phase = self.phase
            self.phase = phase
            self.phase_start_time = self._now()
            self._reset_waypoint_state()
            self.get_logger().info(f"Phase -> {phase.name}")
    
    def _reset_waypoint_state(self):
        """Reset waypoint tracking state."""
        self.reached_since = None
        self.waypoint_start_time = None
        self.heading_aligned_since = None
    
    def _advance_waypoint(self):
        """Advance to next waypoint."""
        self.waypoint_idx += 1
        self._reset_waypoint_state()
        self._reset_boundary_avoidance()
    
    def _is_mapping_stuck(self, now: float) -> bool:
        """Check if mapping appears stuck."""
        if self.phase != MissionPhase.MAPPING:
            return False
        if self.waypoint_idx >= len(self.waypoints):
            return False
        
        tx, ty, _ = self.waypoints[self.waypoint_idx]
        dist = math.hypot(tx - self.position.x, ty - self.position.y)
        speed = self.velocity.speed
        
        threshold_dist = self.config.get('mapping', 'stuck_distance_threshold', default=0.55)
        threshold_speed = self.config.get('mapping', 'stuck_speed_threshold', default=0.05)
        timeout = self.config.get('mapping', 'stuck_timeout', default=10.0)
        
        if dist < threshold_dist or speed > threshold_speed:
            return False
        
        if not hasattr(self, '_stuck_since'):
            self._stuck_since = None
        
        if self._stuck_since is None:
            self._stuck_since = now
            return False
        
        return (now - self._stuck_since) >= timeout
    
    def _update_battery(self, now: float):
        """Update battery estimate."""
        dt = max(0.0, now - self.battery.last_update)
        self.battery.last_update = now
        
        if dt <= 0:
            return
        
        # Compute load factor
        speed = self.velocity.speed
        max_speed = self.config.get('battery', 'max_reasonable_speed', default=3.0)
        speed_load = min(1.0, speed / max(self.config.get('mapping', 'speed_boost', default=1.65), 1e-6))
        dyn_load = max(self.battery.motor_load, speed_load)
        
        if not self.sensors.has_odom:
            dyn_load = 0.0
        
        # Compute drain rate
        idle_rate = self.config.get('battery', 'idle_drain_rate', default=0.0025)
        full_rate = self.config.get('battery', 'full_drain_rate', default=0.0600)
        drain_rate = idle_rate + (full_rate - idle_rate) * dyn_load
        
        self.battery.percent = self._clamp(self.battery.percent - drain_rate * dt, 0.0, 100.0)
        
        # Update dashboard metrics
        alpha = self.config.get('battery', 'smoothing_alpha', default=0.18)
        self.dashboard_speed = (1.0 - alpha) * self.dashboard_speed + alpha * speed
        self.dashboard_distance = (1.0 - alpha) * self.dashboard_distance + alpha * self.position.xy_distance
    
    def _update_coverage(self, now: float):
        """Update coverage grid."""
        if not self.sensors.has_odom:
            return
        
        extent = self.config.get('mapping', 'arena_half_extent', default=8.0)
        resolution = self.config.get('mapping', 'coverage_resolution', default=0.5)
        
        ix = int((self.position.x + extent) / resolution)
        iy = int((self.position.y + extent) / resolution)
        
        # Clamp to grid bounds
        ix = max(0, min(self.coverage_grid.shape[1] - 1, ix))
        iy = max(0, min(self.coverage_grid.shape[0] - 1, iy))
        
        self.coverage_grid[iy, ix] = True
        covered = int(np.count_nonzero(self.coverage_grid))
        self.coverage_percent = 100.0 * float(covered) / float(self.coverage_grid.size)
    
    def _check_comm_loss(self, now: float) -> bool:
        """Check for communication loss."""
        timeout = self.config.get('safety', 'command_timeout', default=1.0)
        
        # Check if manual commands are stale
        if self.manual_mode and (now - self.manual_cmd_time) > timeout:
            self.get_logger().warn("Manual command timeout - zeroing velocity")
            self.manual_velocity = Velocity()
        
        return False
    
    def _check_failsafe_trigger(self):
        """Check and trigger failsafe conditions."""
        if self.failsafe_triggered:
            return
        if not self.config.get('failsafe', 'test_enabled', default=False):
            return
        if self.phase != MissionPhase.MAPPING:
            return
        
        trigger_dist = self.config.get('failsafe', 'trigger_distance', default=2.9)
        dist = self.position.xy_distance
        
        if dist >= trigger_dist:
            self.failsafe_triggered = True
            self.get_logger().warn(f"FAILSAFE triggered at {dist:.2f}m from base")
            self._set_phase(MissionPhase.FAILSAFE_RETURN_HOME)
    
    def _run_manual_control(self, now: float):
        """Execute manual control mode."""
        timeout = self.config.get('manual_control', 'cmd_timeout', default=0.6)
        
        if (now - self.manual_cmd_time) > timeout:
            v = Velocity()
        else:
            v = self.manual_velocity
        
        self._publish_enable(True)
        self._publish_velocity(v.vx, v.vy, v.vz, v.yaw_rate, rate_limit=False)
    
    def _publish_enable(self, enabled: bool):
        """Publish enable message."""
        if not rclpy.ok():
            return
        msg = Bool()
        msg.data = enabled
        self.pub_enable.publish(msg)
    
    def _publish_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float, rate_limit: bool = True):
        """Publish velocity command with optional rate limiting."""
        if not rclpy.ok():
            return
        
        now = self._now()
        
        if rate_limit and self.config.get('command_rate_limiting', 'enable', default=True):
            dt = max(1e-3, now - self.last_cmd_time)
            
            accel_xy = self.config.get('command_rate_limiting', 'xy_acceleration', default=2.2)
            accel_z = self.config.get('command_rate_limiting', 'z_acceleration', default=1.4)
            accel_yaw = self.config.get('command_rate_limiting', 'yaw_acceleration', default=2.0)
            
            self.cmd_velocity.vx += self._clamp(vx - self.cmd_velocity.vx, -accel_xy * dt, accel_xy * dt)
            self.cmd_velocity.vy += self._clamp(vy - self.cmd_velocity.vy, -accel_xy * dt, accel_xy * dt)
            self.cmd_velocity.vz += self._clamp(vz - self.cmd_velocity.vz, -accel_z * dt, accel_z * dt)
            self.cmd_velocity.yaw_rate += self._clamp(yaw_rate - self.cmd_velocity.yaw_rate, -accel_yaw * dt, accel_yaw * dt)
            
            vx, vy, vz, yaw_rate = self.cmd_velocity.vx, self.cmd_velocity.vy, self.cmd_velocity.vz, self.cmd_velocity.yaw_rate
        else:
            self.cmd_velocity = Velocity(vx, vy, vz, yaw_rate)
        
        self.last_cmd_time = now
        
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.pub_twist.publish(msg)
    
    def _publish_scan_tf(self):
        """Publish static transform for LiDAR frame."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "lidar_link"
        tf_msg.transform.translation.z = 0.065
        tf_msg.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf_msg)
    
    def _show_debug_windows(self):
        """Show debug visualization windows."""
        if self.show_windows:
            if self.front_debug is not None:
                cv2.imshow("Front Camera - Yellow Boundary", self.front_debug)
            if self.down_debug is not None:
                cv2.imshow("Belly Camera - Yellow Boundary", self.down_debug)
            
            # Dashboard
            dashboard = self._create_dashboard()
            cv2.imshow("Mission Dashboard", dashboard)
            cv2.waitKey(1)
        
        # Log dashboard periodically
        now = self._now()
        if now - self.last_dashboard_log > 1.0:
            self.last_dashboard_log = now
            self.get_logger().info(
                f"[Dashboard] battery={self.battery.percent:.1f}% speed={self.dashboard_speed:.2f}m/s "
                f"dist={self.dashboard_distance:.2f}m mapped={self.map_known_percent:.1f}% "
                f"covered={self.coverage_percent:.1f}% phase={self.phase.name}"
            )
    
    def _create_dashboard(self) -> np.ndarray:
        """Create dashboard visualization."""
        h, w = 230, 420
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = (24, 24, 24)
        
        # Progress calculation
        progress = 100.0 * float(self.waypoint_idx) / max(len(self.waypoints), 1)
        
        # Battery color
        batt_color = (70, 200, 70)
        if self.battery.percent <= self.config.get('battery', 'low_warn_percent', default=25.0):
            batt_color = (0, 215, 255)
        if self.battery.percent <= self.config.get('battery', 'critical_percent', default=12.0):
            batt_color = (40, 40, 230)
        
        # Draw text
        cv2.putText(img, "Mission Dashboard", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
        
        mode_str = 'MANUAL' if self.manual_mode else 'AUTO'
        cv2.putText(img, f"Mode: {mode_str}   Phase: {self.phase.name}", (12, 48),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.47, (220, 220, 220), 1, cv2.LINE_AA)
        
        cv2.putText(img, f"Speed: {self.dashboard_speed:.2f} m/s", (12, 78),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"Distance: {self.dashboard_distance:.2f} m", (12, 104),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"Mapped: {self.map_known_percent:.1f}%", (12, 130),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"Covered: {self.coverage_percent:.1f}%", (12, 156),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"Progress: {progress:.1f}%", (12, 182),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
        
        # Battery bar
        bar_x, bar_y, bar_w, bar_h = 12, 198, 280, 24
        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (85, 85, 85), 2)
        fill_w = int((self.battery.percent / 100.0) * (bar_w - 4))
        fill_w = max(0, min(bar_w - 4, fill_w))
        if fill_w > 0:
            cv2.rectangle(img, (bar_x + 2, bar_y + 2), (bar_x + 2 + fill_w, bar_y + bar_h - 2), batt_color, -1)
        cv2.putText(img, f"Battery: {self.battery.percent:.1f}%", (bar_x + bar_w + 12, bar_y + 18),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.56, batt_color, 2, cv2.LINE_AA)
        
        return img
    
    def _log_startup_info(self):
        """Log startup information."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("SLAM Boundary Mapping Mission v2.0 (Optimized)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Waypoints: {len(self.waypoints)}")
        self.get_logger().info(f"Control rate: {self.config.get('mission', 'control_rate', default=20.0)} Hz")
        self.get_logger().info(f"Target altitude: {self.config.get('mission', 'target_altitude', default=4.0)} m")
        self.get_logger().info(f"Max XY speed: {self.config.get('velocity_limits', 'max_xy_speed', default=1.45)} m/s")
        self.get_logger().info("Waiting for sensors: odom, scan, cameras...")


def main(args=None):
    rclpy.init(args=args)
    
    # Check for config file argument
    config_path = None
    for i, arg in enumerate(os.sys.argv):
        if arg == '--config' and i + 1 < len(os.sys.argv):
            config_path = os.sys.argv[i + 1]
            break
    
    node = SlamBoundaryMissionOptimized(config_path)
    
    # Use multi-threaded executor for better performance
    executor = MultiThreadedExecutor(num_threads=4)
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
