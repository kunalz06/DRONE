#!/usr/bin/env python3
"""
Common utilities and base classes for drone mission control.
This module provides shared functionality for both simulation and hardware deployments.

Author: Drone Project Team
Version: 2.0.0 (Optimized)
"""

import math
import numpy as np
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, List, Optional, Tuple
import yaml
import os


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
    FAILSAFE_RETURN_HOME_AT_4M = auto()
    FAILSAFE_HOVER_HOME_AT_4M = auto()
    FAILSAFE_ALIGN_YAW_AT_4M = auto()
    FAILSAFE_DESCEND_TO_2M = auto()
    FAILSAFE_HOLD_AT_2M = auto()
    FAILSAFE_ALIGN_YAW_AT_2M = auto()
    FAILSAFE_LAND = auto()
    # Hardware-specific phases
    WAITING_FOR_CHARGE = auto()
    DOCKING = auto()
    SYNCING = auto()


@dataclass
class PIDGains:
    """PID controller gains."""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 10.0


@dataclass
class VelocityCommand:
    """Velocity command with rate limiting."""
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class DroneState:
    """Current state of the drone."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    quat: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    
    @property
    def position(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)
    
    @property
    def speed(self) -> float:
        return math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
    
    @property
    def horizontal_speed(self) -> float:
        return math.sqrt(self.vx**2 + self.vy**2)


@dataclass
class SafetyLimits:
    """Safety limits for the drone."""
    max_xy_speed: float = 1.45
    max_z_speed: float = 0.6
    max_yaw_rate: float = 0.9
    max_distance_from_base: float = 15.0
    min_altitude: float = 0.3
    max_altitude: float = 10.0


class PIDController:
    """PID controller with anti-windup."""
    
    def __init__(self, gains: PIDGains):
        self.gains = gains
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False
    
    def update(self, error: float, dt: float, feedforward: float = 0.0) -> float:
        """Compute PID output."""
        if dt <= 0:
            return feedforward
        
        # Proportional term
        p_term = self.gains.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = clamp(self.integral, -self.gains.integral_limit, self.gains.integral_limit)
        i_term = self.gains.ki * self.integral
        
        # Derivative term (with filtering on first call)
        if not self.initialized:
            self.prev_error = error
            self.initialized = True
        d_term = self.gains.kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        return p_term + i_term + d_term + feedforward
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False


class RateLimiter:
    """Rate limiter for smooth velocity commands."""
    
    def __init__(self, max_accel_xy: float, max_accel_z: float, max_accel_yaw: float):
        self.max_accel_xy = max_accel_xy
        self.max_accel_z = max_accel_z
        self.max_accel_yaw = max_accel_yaw
        self.current_cmd = VelocityCommand()
    
    def update(self, target: VelocityCommand, dt: float) -> VelocityCommand:
        """Apply rate limiting to velocity command."""
        if dt <= 0:
            return self.current_cmd
        
        # Calculate max delta for each axis
        dv_xy = self.max_accel_xy * dt
        dv_z = self.max_accel_z * dt
        dv_yaw = self.max_accel_yaw * dt
        
        # Apply rate limiting
        self.current_cmd.vx += clamp(target.vx - self.current_cmd.vx, -dv_xy, dv_xy)
        self.current_cmd.vy += clamp(target.vy - self.current_cmd.vy, -dv_xy, dv_xy)
        self.current_cmd.vz += clamp(target.vz - self.current_cmd.vz, -dv_z, dv_z)
        self.current_cmd.yaw_rate += clamp(target.yaw_rate - self.current_cmd.yaw_rate, -dv_yaw, dv_yaw)
        
        return self.current_cmd
    
    def reset(self):
        """Reset rate limiter state."""
        self.current_cmd = VelocityCommand()


def clamp(value: float, low: float, high: float) -> float:
    """Clamp value to range [low, high]."""
    return max(low, min(high, value))


def angle_wrap(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_angle(error: float) -> float:
    """Normalize angle error to [-pi, pi]."""
    while error > math.pi:
        error -= 2 * math.pi
    while error < -math.pi:
        error += 2 * math.pi
    return error


def quat_to_yaw(quat: np.ndarray) -> float:
    """Extract yaw from quaternion [x, y, z, w]."""
    x, y, z, w = quat
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion to rotation matrix."""
    qx, qy, qz, qw = quat
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-9:
        return np.eye(3)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    
    return np.array([
        [1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw), 2.0*(qx*qz + qy*qw)],
        [2.0*(qx*qy + qz*qw), 1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw)],
        [2.0*(qx*qz - qy*qw), 2.0*(qy*qz + qx*qw), 1.0 - 2.0*(qx*qx + qy*qy)]
    ], dtype=np.float64)


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert roll-pitch-yaw to rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ], dtype=np.float64)


def world_to_body(vx_world: float, vy_world: float, yaw: float) -> Tuple[float, float]:
    """Transform velocity from world frame to body frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vx_world + s * vy_world, -s * vx_world + c * vy_world)


def body_to_world(vx_body: float, vy_body: float, yaw: float) -> Tuple[float, float]:
    """Transform velocity from body frame to world frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vx_body - s * vy_body, s * vx_body + c * vy_body)


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate 2D Euclidean distance."""
    return math.hypot(x2 - x1, y2 - y1)


def distance_3d(x1: float, y1: float, z1: float, x2: float, y2: float, z2: float) -> float:
    """Calculate 3D Euclidean distance."""
    return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)


def normalize_vector(vx: float, vy: float) -> Tuple[float, float, float]:
    """Normalize a 2D vector, return (nx, ny, magnitude)."""
    mag = math.hypot(vx, vy)
    if mag < 1e-6:
        return (0.0, 0.0, 0.0)
    return (vx / mag, vy / mag, mag)


def load_yaml_config(filepath: str) -> dict:
    """Load YAML configuration file."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Configuration file not found: {filepath}")
    
    with open(filepath, 'r') as f:
        config = yaml.safe_load(f)
    
    return config


def build_sweep_waypoints(
    arena_half_extent: float,
    edge_margin: float,
    lane_spacing: float,
    target_alt: float
) -> List[Tuple[float, float, float]]:
    """
    Build boundary-aligned sweep waypoints for arena mapping.
    Returns list of (x, y, z) waypoints.
    """
    x_min = -arena_half_extent + edge_margin
    x_max = arena_half_extent - edge_margin
    y_min = -arena_half_extent + edge_margin
    y_max = arena_half_extent - edge_margin
    
    lane_spacing = max(0.25, lane_spacing)
    span_y = max(0.1, y_max - y_min)
    lane_count = max(1, int(math.floor(span_y / lane_spacing)) + 1)
    lane_step = 0.0 if lane_count == 1 else span_y / float(lane_count - 1)
    
    waypoints = []
    for lane_idx in range(lane_count):
        y_lane = y_min + lane_idx * lane_step
        
        # Alternate direction for efficiency (lawn-mower pattern)
        if lane_idx % 2 == 0:
            x_start, x_end = x_min, x_max
        else:
            x_start, x_end = x_max, x_min
        
        # Add start waypoint if not duplicate
        start_wp = (x_start, y_lane, target_alt)
        if not waypoints or any(abs(start_wp[i] - waypoints[-1][i]) > 1e-6 for i in range(3)):
            waypoints.append(start_wp)
        
        # Add end waypoint if not duplicate
        end_wp = (x_end, y_lane, target_alt)
        if any(abs(end_wp[i] - waypoints[-1][i]) > 1e-6 for i in range(3)):
            waypoints.append(end_wp)
    
    return waypoints


class YellowDetector:
    """
    Optimized yellow color detector using HSV thresholding.
    Used for boundary line detection.
    """
    
    def __init__(
        self,
        h_min: int = 18, h_max: int = 38,
        s_min: int = 80, s_max: int = 255,
        v_min: int = 80, v_max: int = 255
    ):
        self.lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
        self.upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
        self.kernel = np.ones((3, 3), np.uint8)
    
    def detect(self, frame: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Detect yellow pixels in frame.
        Returns (yellow_ratio, visualization_frame).
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        
        ratio = float(np.count_nonzero(mask)) / float(mask.size)
        
        vis = frame.copy()
        vis[mask > 0] = (0, 255, 255)  # Highlight yellow in BGR
        
        return ratio, vis
    
    def detect_with_centroid(self, frame: np.ndarray) -> Tuple[float, np.ndarray, Optional[Tuple[int, int]]]:
        """
        Detect yellow pixels and compute centroid.
        Returns (yellow_ratio, visualization_frame, centroid).
        """
        ratio, vis = self.detect(frame)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        
        centroid = None
        m = cv2.moments(mask, binaryImage=True)
        if m["m00"] > 1e-6:
            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])
            centroid = (cx, cy)
        
        return ratio, vis, centroid


# Import cv2 at module level (with fallback for headless operation)
try:
    import cv2
except ImportError:
    cv2 = None  # type: ignore
    print("[WARN] OpenCV not available. Image processing features disabled.")


class PerformanceMonitor:
    """Simple performance monitor for timing code sections."""
    
    def __init__(self):
        self.timings = {}
        self.counts = {}
    
    def start(self, name: str):
        """Start timing a section."""
        import time
        self.timings[name] = {'start': time.perf_counter()}
    
    def end(self, name: str):
        """End timing a section."""
        import time
        if name in self.timings:
            elapsed = time.perf_counter() - self.timings[name]['start']
            if name not in self.counts:
                self.counts[name] = {'total': 0.0, 'count': 0}
            self.counts[name]['total'] += elapsed
            self.counts[name]['count'] += 1
    
    def get_stats(self, name: str) -> dict:
        """Get statistics for a section."""
        if name not in self.counts:
            return {'avg': 0.0, 'count': 0}
        c = self.counts[name]
        return {'avg': c['total'] / c['count'], 'count': c['count']}
    
    def get_report(self) -> str:
        """Get performance report."""
        lines = ["Performance Report:"]
        for name in sorted(self.counts.keys()):
            stats = self.get_stats(name)
            lines.append(f"  {name}: avg={stats['avg']*1000:.2f}ms, count={stats['count']}")
        return "\n".join(lines)
