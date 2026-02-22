#!/usr/bin/env python3
import math
import os
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from actuator_msgs.msg import Actuators
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class SlamBoundaryMission(Node):
    def __init__(self):
        super().__init__("slam_boundary_mapping_mission")

        self.target_alt_m = 4.0
        self.control_hz = 20.0
        self.base_xy = (0.0, 0.0)
        self.max_xy_speed = 1.45
        self.max_z_speed = 0.6
        self.auto_max_yaw_rate = 0.9
        self.auto_yaw_kp = 1.4
        self.auto_yaw_align_tolerance_rad = math.radians(6.0)
        self.auto_yaw_hover_before_move_s = 0.15
        self.auto_yaw_align_timeout_s = 1.8
        self.auto_yaw_progress_ratio_while_aligning = 0.35
        self.auto_yaw_lateral_limit_while_aligning = 0.20
        self.mapping_speed_boost = 1.65
        self.mapping_precision_radius_m = 1.4
        self.mapping_precision_min_speed = 0.55
        self.heading_activation_dist_m = 0.35
        self.cmd_vel_is_body_frame = True
        self.arena_half_extent_m = 8.0
        self.mapping_edge_margin_m = 1.2
        self.sweep_lane_spacing_m = 0.9
        self.boundary_trigger_extent_m = 7.2
        self.waypoint_timeout_s = 22.0
        self.home_hover_s = 2.0
        self.mid_alt_m = 2.0
        self.mid_alt_hover_s = 2.0
        self.landing_z_m = 0.20
        self.slow_descend_max_z_speed = 0.16
        self.landing_max_z_speed = 0.12
        self.base_arrow_yaw_rad = 0.0
        self.landing_yaw_rad = self.base_arrow_yaw_rad
        self.failsafe_test_enabled = False
        self.failsafe_trigger_distance_m = 2.9
        self.failed_motor_index = 3  # 0-based: rotor_3 (4th motor)
        self.failsafe_three_motor_xy_speed = 0.45
        self.failsafe_three_motor_z_speed = 0.20
        self.failsafe_three_motor_land_z_speed = 0.08
        self.esc_current_limit_a = 40.0
        self.manual_cmd_timeout_s = 0.6
        self.manual_max_xy_speed = 1.4
        self.manual_max_z_speed = 0.7
        self.manual_max_yaw_rate = 1.2
        self.tof_scan_topic = "/tof_scan"
        self.tof_min_range_m = 0.05
        self.tof_max_range_m = 6.0
        self.tof_full_weight_range_m = 2.5
        self.tof_blend_weight = 0.65
        self.tof_timeout_s = 0.5
        self.coverage_resolution_m = 0.50
        self.terrain_scan_topic = "/terrain_scan"
        self.terrain_cloud_topic = "/terrain_point_cloud"
        self.terrain_point_stride = 2
        self.terrain_lidar_pitch_rad = -0.52
        self.terrain_lidar_offset_m = np.array([0.0, 0.0, 0.065], dtype=np.float64)
        self.battery_percent = 100.0
        self.battery_low_warn_percent = 25.0
        self.battery_critical_percent = 12.0
        self.battery_idle_drain_pct_per_s = 0.0025
        self.battery_full_drain_pct_per_s = 0.0600
        self.max_reasonable_speed_mps = 3.0
        self.max_reasonable_distance_m = 12.0
        self.dashboard_smoothing_alpha = 0.18
        self.rate_limit_cmds = True
        self.auto_cmd_accel_xy = 2.2
        self.auto_cmd_accel_z = 1.4
        self.auto_cmd_accel_yaw = 2.0

        self.front_yellow_threshold = 0.010
        self.down_yellow_threshold = 0.010
        self.yellow_hold_s = 0.5
        self.avoid_duration_s = 1.2
        self.down_repulse_max_speed = 0.65
        self.down_repulse_full_ratio = 0.10
        self.boundary_guard_enter_strength = 0.08
        self.boundary_guard_exit_strength = 0.03
        self.boundary_hover_s = 0.30
        self.boundary_retreat_base_speed = 0.35
        self.boundary_retreat_gain = 0.35
        self.boundary_move_yaw_tolerance_rad = math.radians(6.0)
        self.boundary_min_escape_speed = 0.25
        self.boundary_max_lateral_speed = 0.20
        self.boundary_avoid_max_duration_s = 6.0
        self.mapping_stuck_speed_mps = 0.05
        self.mapping_stuck_distance_m = 0.55
        self.mapping_stuck_timeout_s = 10.0
        self.obstacle_avoid_dist_m = 2.2
        self.obstacle_repulse_gain = 0.85
        self.last_scan_ranges = None
        self.last_scan_angles = None

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.pub_enable = self.create_publisher(Bool, "/drone_enable", 10)
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_motor_cmd = self.create_publisher(Actuators, "/motor_speed_cmd_out", 10)
        self.pub_terrain_cloud = self.create_publisher(PointCloud2, self.terrain_cloud_topic, 10)

        self.sub_odom = self.create_subscription(Odometry, "/odom", self._odom_cb, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.sub_terrain_scan = self.create_subscription(
            LaserScan, self.terrain_scan_topic, self._terrain_scan_cb, qos_profile_sensor_data
        )
        self.sub_front = self.create_subscription(Image, "/front_camera/image_raw", self._front_cb, qos_profile_sensor_data)
        self.sub_down = self.create_subscription(Image, "/down_camera/image_raw", self._down_cb, qos_profile_sensor_data)
        self.sub_motor_cmd = self.create_subscription(
            Actuators, "/motor_speed_cmd_in", self._motor_cmd_cb, qos_profile_sensor_data
        )
        self.sub_manual_mode = self.create_subscription(Bool, "/manual_mode", self._manual_mode_cb, 10)
        self.sub_manual_cmd = self.create_subscription(Twist, "/manual_cmd_vel", self._manual_cmd_cb, 10)
        self.sub_manual_rtb = self.create_subscription(Bool, "/manual_rtb", self._manual_rtb_cb, 10)
        self.sub_tof_scan = self.create_subscription(LaserScan, self.tof_scan_topic, self._tof_scan_cb, qos_profile_sensor_data)
        self.sub_map = self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)

        self.has_odom = False
        self.has_scan = False
        self.has_terrain_scan = False
        self.has_front = False
        self.has_down = False
        self.has_motor_cmd = False
        self.tof_valid = False
        self.tof_range_m = 0.0
        self.tof_last_update_s = -1.0
        self.map_known_percent = 0.0
        self.map_last_update_s = -1.0
        self.scan_frame_id = "lidar_link"
        self.published_static_scan_frame_id = ""
        self.odom_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self.terrain_sensor_rot = self._rpy_to_rot_matrix(0.0, self.terrain_lidar_pitch_rad, 0.0)
        self.terrain_point_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.front_yellow_ratio = 0.0
        self.down_yellow_ratio = 0.0
        self.front_seen_time = -1.0
        self.down_seen_time = -1.0
        self.down_repulse_body_x = 0.0
        self.down_repulse_body_y = 0.0
        self.down_repulse_strength = 0.0

        self.show_windows = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        self.front_debug = None
        self.down_debug = None

        self.boundary_avoid_until = -1.0
        self.boundary_hover_until = -1.0
        self.boundary_avoid_start_s = -1.0
        self.last_boundary_log_s = -1.0
        self.heading_aligned_since = None
        self.heading_align_wait_start_s = None

        self.phase = "WAIT_SENSORS"
        self.phase_start = self._now_s()
        self.reached_since = None
        self.failsafe_triggered = False
        self.last_motor_cmd_warn_s = -1.0
        self.manual_mode_enabled = False
        self.last_manual_cmd_s = -1.0
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_vz = 0.0
        self.manual_yaw_rate = 0.0
        self.rtb_requested = False
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_yaw_rate = 0.0
        self.last_cmd_time_s = self._now_s()
        self.motor_load = 0.0
        self.last_battery_update_s = self._now_s()
        self.last_dashboard_log_s = -1.0
        self.dashboard_speed_mps = 0.0
        self.dashboard_distance_m = 0.0
        self.arena_coverage_percent = 0.0
        self.last_coverage_update_s = -1.0
        self.mapping_stuck_since = None

        self.waypoints = self._build_mapping_waypoints()
        self.waypoint_idx = 0
        self.active_waypoint_start_s = None
        self.coverage_grid = self._init_coverage_grid()

        self.control_timer = self.create_timer(1.0 / self.control_hz, self._control_tick)

        self.get_logger().info("SLAM mapping mission node started.")
        self.get_logger().info(
            f"Boundary-aligned sweep ready with {len(self.waypoints)} waypoints "
            f"(start corner: bottom-left, lane spacing {self.sweep_lane_spacing_m:.2f} m)."
        )
        self.get_logger().info(f"ESC controller active: {self.esc_current_limit_a:.0f}A per motor channel.")
        if self.failsafe_test_enabled:
            self.get_logger().info(
                f"Failsafe test armed: motor {self.failed_motor_index + 1} will stop at distance >= {self.failsafe_trigger_distance_m:.1f} m."
            )
        else:
            self.get_logger().info("Failsafe logic is enabled but test trigger is disabled for mapping runs.")
        self.get_logger().info("Waiting for odom, lidar scan, camera streams, and motor command bridge...")
        self.get_logger().info("Manual override topics ready: /manual_mode (Bool), /manual_cmd_vel (Twist).")
        self.get_logger().info("Manual RTB topic ready: /manual_rtb (Bool).")
        self.get_logger().info("Autonomous heading control enabled: hover + yaw before directional movement.")
        self.get_logger().info(
            f"Heading fallback enabled: after {self.auto_yaw_align_timeout_s:.1f}s, mission allows reduced XY motion while yaw converges."
        )
        self.get_logger().info(
            f"Autonomous velocity command frame: {'body' if self.cmd_vel_is_body_frame else 'world'}"
        )
        self.get_logger().info(
            f"Mapping speed profile enabled: up to {self.mapping_speed_boost:.2f} m/s with precision slowdown inside {self.mapping_precision_radius_m:.1f} m."
        )
        self.get_logger().info("Dashboard battery uses an onboard estimate derived from motion + motor load.")
        self.get_logger().info(
            f"Terrain mapping point cloud enabled: {self.terrain_scan_topic} -> {self.terrain_cloud_topic}"
        )
        self.get_logger().info(
            f"ToF altitude hold enabled on {self.tof_scan_topic} (blend={self.tof_blend_weight:.2f}, max={self.tof_max_range_m:.1f} m)."
        )
        self.get_logger().info("Belly camera repulsion enabled: yellow-line vector pushes opposite direction with proximity gain.")
        self.get_logger().info("Boundary guard enabled: hover + inward retreat when yellow proximity exceeds threshold.")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _build_mapping_waypoints(self) -> List[Tuple[float, float, float]]:
        z = self.target_alt_m
        x_min = -self.arena_half_extent_m + self.mapping_edge_margin_m
        x_max = self.arena_half_extent_m - self.mapping_edge_margin_m
        y_min = -self.arena_half_extent_m + self.mapping_edge_margin_m
        y_max = self.arena_half_extent_m - self.mapping_edge_margin_m
        lane_spacing = max(0.25, self.sweep_lane_spacing_m)
        span_y = max(0.1, y_max - y_min)
        lane_count = max(1, int(math.floor(span_y / lane_spacing)) + 1)
        lane_step = 0.0 if lane_count == 1 else span_y / float(lane_count - 1)

        waypoints: List[Tuple[float, float, float]] = []
        for lane_idx in range(lane_count):
            y_lane = y_min + lane_idx * lane_step
            if lane_idx % 2 == 0:
                x_start, x_end = x_min, x_max
            else:
                x_start, x_end = x_max, x_min
            start_wp = (x_start, y_lane, z)
            end_wp = (x_end, y_lane, z)
            if not waypoints or any(abs(start_wp[i] - waypoints[-1][i]) > 1e-6 for i in range(3)):
                waypoints.append(start_wp)
            if any(abs(end_wp[i] - waypoints[-1][i]) > 1e-6 for i in range(3)):
                waypoints.append(end_wp)
        return waypoints

    def _init_coverage_grid(self) -> np.ndarray:
        span = max(0.1, 2.0 * self.arena_half_extent_m)
        res = max(0.1, self.coverage_resolution_m)
        cells = max(1, int(math.ceil(span / res)))
        return np.zeros((cells, cells), dtype=bool)

    def _odom_cb(self, msg: Odometry):
        self.has_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        q = msg.pose.pose.orientation
        self.odom_quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        tf_msg = TransformStamped()
        tf_stamp = msg.header.stamp
        if tf_stamp.sec == 0 and tf_stamp.nanosec == 0:
            tf_stamp = self.get_clock().now().to_msg()
        tf_msg.header.stamp = tf_stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def _scan_cb(self, msg: LaserScan):
        self.has_scan = True
        if msg.header.frame_id:
            self.scan_frame_id = msg.header.frame_id
        
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        self.last_scan_ranges = ranges[valid]
        angles = msg.angle_min + np.arange(len(msg.ranges), dtype=np.float64) * msg.angle_increment
        self.last_scan_angles = angles[valid]

    def _tof_scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            self.tof_valid = False
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        finite = ranges[np.isfinite(ranges)]
        if finite.size == 0:
            self.tof_valid = False
            return

        valid = finite[(finite >= msg.range_min) & (finite <= msg.range_max)]
        if valid.size == 0:
            self.tof_valid = False
            return

        self.tof_range_m = float(np.min(valid))
        self.tof_last_update_s = self._now_s()
        self.tof_valid = True

    def _map_cb(self, msg: OccupancyGrid):
        if not msg.data:
            return
        data = np.asarray(msg.data, dtype=np.int16)
        total = data.size
        if total == 0:
            return
        known = int(np.count_nonzero(data >= 0))
        self.map_known_percent = 100.0 * float(known) / float(total)
        self.map_last_update_s = self._now_s()

    @staticmethod
    def _rpy_to_rot_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _quat_to_rot_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
        qx, qy, qz, qw = quat_xyzw
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9:
            return np.eye(3, dtype=np.float64)
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm
        return np.array(
            [
                [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
                [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
                [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
            ],
            dtype=np.float64,
        )

    def _terrain_scan_cb(self, msg: LaserScan):
        self.has_terrain_scan = True
        if not self.has_odom:
            return

        if not msg.ranges or abs(msg.angle_increment) < 1e-9:
            return

        stride = max(1, int(self.terrain_point_stride))
        ranges = np.asarray(msg.ranges, dtype=np.float64)[::stride]
        ray_idx = np.arange(0, len(msg.ranges), stride, dtype=np.float64)
        angles = msg.angle_min + ray_idx * msg.angle_increment

        valid = np.isfinite(ranges)
        valid = np.logical_and(valid, ranges >= msg.range_min)
        valid = np.logical_and(valid, ranges <= msg.range_max)
        if not np.any(valid):
            return

        ranges = ranges[valid]
        angles = angles[valid]

        points_sensor = np.vstack(
            (
                ranges * np.cos(angles),
                ranges * np.sin(angles),
                np.zeros_like(ranges),
            )
        )

        points_base = self.terrain_sensor_rot @ points_sensor
        points_base += self.terrain_lidar_offset_m.reshape(3, 1)

        rot_odom_base = self._quat_to_rot_matrix(self.odom_quat)
        base_pos = np.array([self.x, self.y, self.z], dtype=np.float64).reshape(3, 1)
        points_odom = rot_odom_base @ points_base + base_pos

        points_xyz = points_odom.T.astype(np.float32)
        if points_xyz.shape[0] == 0:
            return

        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = "odom"
        cloud.height = 1
        cloud.width = points_xyz.shape[0]
        cloud.fields = self.terrain_point_fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = points_xyz.tobytes()
        self.pub_terrain_cloud.publish(cloud)

    def _motor_cmd_cb(self, msg: Actuators):
        self.has_motor_cmd = True

        out = Actuators()
        out.header = msg.header
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.normalized = list(msg.normalized)

        if self.failsafe_triggered:
            if len(out.velocity) > self.failed_motor_index:
                out.velocity[self.failed_motor_index] = 0.0
            if len(out.normalized) > self.failed_motor_index:
                out.normalized[self.failed_motor_index] = 0.0

        if len(out.normalized) > 0:
            mean_norm = float(np.mean(np.abs(np.asarray(out.normalized, dtype=np.float64))))
            self.motor_load = self._clamp(mean_norm, 0.0, 1.0)
        elif len(out.velocity) > 0:
            vel = np.asarray(out.velocity, dtype=np.float64)
            finite = vel[np.isfinite(vel)]
            if finite.size > 0:
                inferred = float(np.mean(np.abs(finite)) / max(1.0, np.max(np.abs(finite))))
                self.motor_load = self._clamp(inferred, 0.0, 1.0)

        if not rclpy.ok():
            return
        try:
            self.pub_motor_cmd.publish(out)
        except Exception:
            pass

    def _manual_mode_cb(self, msg: Bool):
        prev = self.manual_mode_enabled
        self.manual_mode_enabled = bool(msg.data)
        if self.manual_mode_enabled != prev:
            mode_str = "MANUAL" if self.manual_mode_enabled else "AUTONOMOUS"
            self.get_logger().warn(f"Control mode -> {mode_str}")
            self.reached_since = None
            self.active_waypoint_start_s = None
            self.heading_aligned_since = None
            self.heading_align_wait_start_s = None
        if not self.manual_mode_enabled:
            self.manual_vx = 0.0
            self.manual_vy = 0.0
            self.manual_vz = 0.0
            self.manual_yaw_rate = 0.0

    def _manual_rtb_cb(self, msg: Bool):
        if not msg.data:
            return
        self.rtb_requested = True
        self.manual_mode_enabled = False
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_vz = 0.0
        self.manual_yaw_rate = 0.0
        self.last_manual_cmd_s = self._now_s()
        self.get_logger().warn("Manual RTB requested: switching to return, yaw align, and landing sequence.")

    def _manual_cmd_cb(self, msg: Twist):
        self.last_manual_cmd_s = self._now_s()
        self.manual_vx = self._clamp(msg.linear.x, -self.manual_max_xy_speed, self.manual_max_xy_speed)
        self.manual_vy = self._clamp(msg.linear.y, -self.manual_max_xy_speed, self.manual_max_xy_speed)
        self.manual_vz = self._clamp(msg.linear.z, -self.manual_max_z_speed, self.manual_max_z_speed)
        self.manual_yaw_rate = self._clamp(msg.angular.z, -self.manual_max_yaw_rate, self.manual_max_yaw_rate)

    def _front_cb(self, msg: Image):
        ratio, vis = self._yellow_ratio_and_vis(msg)
        self.has_front = True
        self.front_yellow_ratio = ratio
        if ratio > self.front_yellow_threshold:
            self.front_seen_time = self._now_s()
        self.front_debug = vis

    def _down_cb(self, msg: Image):
        ratio, vis, rep_bx, rep_by, rep_strength = self._down_yellow_vector_and_vis(msg)
        self.has_down = True
        self.down_yellow_ratio = ratio
        if ratio > self.down_yellow_threshold:
            self.down_seen_time = self._now_s()
        self.down_repulse_body_x = rep_bx
        self.down_repulse_body_y = rep_by
        self.down_repulse_strength = rep_strength
        self.down_debug = vis

    @staticmethod
    def _yellow_mask(frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([18, 80, 80], dtype=np.uint8)
        upper = np.array([38, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def _yellow_ratio_and_vis(self, msg: Image) -> Tuple[float, Optional[np.ndarray]]:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"Image decode failed: {exc}")
            return 0.0, None

        mask = self._yellow_mask(frame)

        ratio = float(np.count_nonzero(mask)) / float(mask.size)

        vis = frame.copy()
        vis[mask > 0] = (0, 255, 255)
        cv2.putText(
            vis,
            f"yellow_ratio={ratio:.3f}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return ratio, vis

    def _down_yellow_vector_and_vis(self, msg: Image) -> Tuple[float, Optional[np.ndarray], float, float, float]:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"Image decode failed: {exc}")
            return 0.0, None, 0.0, 0.0, 0.0

        mask = self._yellow_mask(frame)
        ratio = float(np.count_nonzero(mask)) / float(mask.size)

        h, w = mask.shape
        cx0 = 0.5 * float(w - 1)
        cy0 = 0.5 * float(h - 1)

        repulse_body_x = 0.0
        repulse_body_y = 0.0
        repulse_strength = 0.0
        repel_img_x = 0.0
        repel_img_y = 0.0
        centroid = None

        m = cv2.moments(mask, binaryImage=True)
        if m["m00"] > 1e-6:
            cx = float(m["m10"] / m["m00"])
            cy = float(m["m01"] / m["m00"])
            centroid = (int(cx), int(cy))

            off_x = (cx - cx0) / max(1.0, 0.5 * float(w))
            off_y = (cy - cy0) / max(1.0, 0.5 * float(h))

            repel_img_x = -off_x
            repel_img_y = -off_y
            n = math.hypot(repel_img_x, repel_img_y)
            if n > 1e-6:
                repel_img_x /= n
                repel_img_y /= n

                # Assumes top of belly image is forward body-x and right of image is body-right.
                body_dir_x = -repel_img_y
                body_dir_y = -repel_img_x

                repulse_strength = self._clamp(
                    (ratio - self.down_yellow_threshold) / max(1e-6, self.down_repulse_full_ratio - self.down_yellow_threshold),
                    0.0,
                    1.0,
                )
                speed = self.down_repulse_max_speed * repulse_strength
                repulse_body_x = speed * body_dir_x
                repulse_body_y = speed * body_dir_y

        vis = frame.copy()
        vis[mask > 0] = (0, 255, 255)

        center_pt = (int(cx0), int(cy0))
        cv2.circle(vis, center_pt, 5, (255, 255, 255), -1, cv2.LINE_AA)
        if centroid is not None:
            cv2.circle(vis, centroid, 6, (0, 0, 255), 2, cv2.LINE_AA)

        arrow_len = int(35 + 160 * repulse_strength)
        arrow_end = (
            int(center_pt[0] + repel_img_x * arrow_len),
            int(center_pt[1] + repel_img_y * arrow_len),
        )
        if repulse_strength > 0.0:
            cv2.arrowedLine(vis, center_pt, arrow_end, (0, 0, 255), 3, cv2.LINE_AA, tipLength=0.25)

        cv2.putText(
            vis,
            f"yellow_ratio={ratio:.3f} repel={repulse_strength:.2f}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            vis,
            f"repel_body vx={repulse_body_x:.2f} vy={repulse_body_y:.2f}",
            (20, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return ratio, vis, repulse_body_x, repulse_body_y, repulse_strength

    @staticmethod
    def _clamp(val: float, low: float, high: float) -> float:
        return max(low, min(high, val))

    @staticmethod
    def _angle_wrap(rad: float) -> float:
        return math.atan2(math.sin(rad), math.cos(rad))

    def _tof_is_valid(self, now_s: float) -> bool:
        if not self.tof_valid:
            return False
        if self.tof_range_m <= self.tof_min_range_m:
            return False
        if self.tof_range_m >= self.tof_max_range_m:
            return False
        if (now_s - self.tof_last_update_s) > self.tof_timeout_s:
            return False
        return True

    def _tof_weight(self) -> float:
        rng = self.tof_range_m
        if rng <= 0.0:
            return 0.0
        if rng <= self.tof_full_weight_range_m:
            return self.tof_blend_weight
        if rng >= self.tof_max_range_m:
            return 0.0
        scale = 1.0 - (rng - self.tof_full_weight_range_m) / max(1e-6, self.tof_max_range_m - self.tof_full_weight_range_m)
        return self.tof_blend_weight * self._clamp(scale, 0.0, 1.0)

    def _compute_vz(self, z_ref: float, z_speed_limit: float, now_s: float) -> float:
        ez = z_ref - self.z
        if self._tof_is_valid(now_s):
            ez_tof = z_ref - self.tof_range_m
            w = self._tof_weight()
            ez = (1.0 - w) * ez + w * ez_tof
        return self._clamp(0.9 * ez, -z_speed_limit, z_speed_limit)

    def _update_coverage(self, now_s: float):
        if not self.has_odom:
            return
        if (now_s - self.last_coverage_update_s) < 0.20:
            return
        self.last_coverage_update_s = now_s

        grid = self.coverage_grid
        if grid.size == 0:
            return
        res = max(0.1, self.coverage_resolution_m)
        ix = int((self.x + self.arena_half_extent_m) / res)
        iy = int((self.y + self.arena_half_extent_m) / res)
        
        # Cap indices to be within valid grid range
        ix = max(0, min(grid.shape[1] - 1, ix))
        iy = max(0, min(grid.shape[0] - 1, iy))
        
        grid[iy, ix] = True
        covered = int(np.count_nonzero(grid))
        self.arena_coverage_percent = 100.0 * float(covered) / float(grid.size)

    def _heading_alignment_gate(self, desired_yaw: float, now_s: float) -> Tuple[bool, float]:
        yaw_err = self._angle_wrap(desired_yaw - self.yaw)
        yaw_rate = self._clamp(
            self.auto_yaw_kp * yaw_err,
            -self.auto_max_yaw_rate,
            self.auto_max_yaw_rate,
        )
        yaw_ok = abs(yaw_err) <= self.auto_yaw_align_tolerance_rad
        if yaw_ok:
            if self.heading_aligned_since is None:
                self.heading_aligned_since = now_s
            heading_ready = (now_s - self.heading_aligned_since) >= self.auto_yaw_hover_before_move_s
        else:
            self.heading_aligned_since = None
            heading_ready = False
        return heading_ready, yaw_rate

    def _down_repulsion_world_vector(self) -> Tuple[float, float]:
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        vx = c * self.down_repulse_body_x - s * self.down_repulse_body_y
        vy = s * self.down_repulse_body_x + c * self.down_repulse_body_y
        return vx, vy

    @staticmethod
    def _normalize_xy(vx: float, vy: float) -> Tuple[float, float, float]:
        mag = math.hypot(vx, vy)
        if mag < 1e-6:
            return 0.0, 0.0, 0.0
        return vx / mag, vy / mag, mag

    def _boundary_retreat_vector_world(self) -> Tuple[float, float]:
        in_x = self.base_xy[0] - self.x
        in_y = self.base_xy[1] - self.y
        in_dx, in_dy, in_mag = self._normalize_xy(in_x, in_y)
        if in_mag < 1e-6:
            rep_vx, rep_vy = self._down_repulsion_world_vector()
            dir_x, dir_y, dir_mag = self._normalize_xy(rep_vx, rep_vy)
            if dir_mag < 1e-6:
                dir_x, dir_y = 1.0, 0.0
        else:
            dir_x, dir_y = in_dx, in_dy

        strength = max(self.down_repulse_strength, self.down_yellow_ratio)
        speed = self._clamp(
            self.boundary_retreat_base_speed + self.boundary_retreat_gain * strength,
            self.boundary_retreat_base_speed,
            self.max_xy_speed,
        )
        return speed * dir_x, speed * dir_y

    def _world_to_body_xy(self, vx_world: float, vy_world: float) -> Tuple[float, float]:
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        vx_body = c * vx_world + s * vy_world
        vy_body = -s * vx_world + c * vy_world
        return vx_body, vy_body

    def _compose_xy_with_heading_lock(self, desired_vx: float, desired_vy: float, now_s: float, align_heading: bool, speed_limit: float) -> Tuple[float, float, float]:
        desired_speed = math.hypot(desired_vx, desired_vy)
        if desired_speed < 1e-5:
            self.heading_aligned_since = None
            self.heading_align_wait_start_s = None
            return 0.0, 0.0, 0.0

        desired_cmd_vx, desired_cmd_vy = desired_vx, desired_vy
        if self.cmd_vel_is_body_frame:
            desired_cmd_vx, desired_cmd_vy = self._world_to_body_xy(desired_vx, desired_vy)

        if not align_heading:
            self.heading_aligned_since = None
            self.heading_align_wait_start_s = None
            return desired_cmd_vx, desired_cmd_vy, 0.0

        if self.heading_align_wait_start_s is None:
            self.heading_align_wait_start_s = now_s
        target_yaw = math.atan2(desired_vy, desired_vx)
        heading_ready, yaw_rate = self._heading_alignment_gate(target_yaw, now_s)
        if not heading_ready:
            align_wait_s = now_s - self.heading_align_wait_start_s
            if align_wait_s < self.auto_yaw_align_timeout_s:
                return 0.0, 0.0, yaw_rate

            progress_ratio = self._clamp(self.auto_yaw_progress_ratio_while_aligning, 0.0, 1.0)
            if self.cmd_vel_is_body_frame:
                # Keep progressing when yaw lock cannot settle, but cap lateral drift.
                vx_progress = self._clamp(desired_cmd_vx * progress_ratio, -speed_limit, speed_limit)
                vy_progress = self._clamp(
                    desired_cmd_vy * progress_ratio,
                    -self.auto_yaw_lateral_limit_while_aligning,
                    self.auto_yaw_lateral_limit_while_aligning,
                )
                return vx_progress, vy_progress, yaw_rate

            return desired_cmd_vx * progress_ratio, desired_cmd_vy * progress_ratio, yaw_rate

        self.heading_align_wait_start_s = None

        if self.cmd_vel_is_body_frame:
            # Enforce zero lateral drift in autonomous mode: move only along body-x once yaw is aligned.
            forward_speed = self._clamp(desired_cmd_vx, 0.0, speed_limit)
            return forward_speed, 0.0, yaw_rate

        nose_x = math.cos(self.yaw)
        nose_y = math.sin(self.yaw)
        dir_x = desired_vx / max(desired_speed, 1e-6)
        dir_y = desired_vy / max(desired_speed, 1e-6)
        forward_alignment = max(0.0, nose_x * dir_x + nose_y * dir_y)
        speed_cmd = desired_speed * forward_alignment
        return speed_cmd * nose_x, speed_cmd * nose_y, yaw_rate

    def _publish_enable(self, enabled: bool):
        if not rclpy.ok():
            return
        msg = Bool()
        msg.data = enabled
        try:
            self.pub_enable.publish(msg)
        except Exception:
            pass

    def _publish_twist(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0, *, rate_limit: Optional[bool] = None):
        if not rclpy.ok():
            return
        if rate_limit is None:
            rate_limit = self.rate_limit_cmds and (not self.manual_mode_enabled)
        
        now = self._now_s()
        dt = max(1e-3, now - self.last_cmd_time_s)
        self.last_cmd_time_s = now

        if rate_limit:
            dv_xy = self.auto_cmd_accel_xy * dt
            dv_z = self.auto_cmd_accel_z * dt
            dv_yaw = self.auto_cmd_accel_yaw * dt
            self.cmd_vx += self._clamp(vx - self.cmd_vx, -dv_xy, dv_xy)
            self.cmd_vy += self._clamp(vy - self.cmd_vy, -dv_xy, dv_xy)
            self.cmd_vz += self._clamp(vz - self.cmd_vz, -dv_z, dv_z)
            self.cmd_yaw_rate += self._clamp(yaw_rate - self.cmd_yaw_rate, -dv_yaw, dv_yaw)
            vx, vy, vz, yaw_rate = self.cmd_vx, self.cmd_vy, self.cmd_vz, self.cmd_yaw_rate
        else:
            # Sync internal cmd state when not rate-limiting (e.g. manual mode) 
            # to prevent jumps when returning to auto mode.
            self.cmd_vx = vx
            self.cmd_vy = vy
            self.cmd_vz = vz
            self.cmd_yaw_rate = yaw_rate

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        try:
            self.pub_twist.publish(msg)
        except Exception:
            pass


    def _run_manual_override(self, now_s: float):
        stale = (now_s - self.last_manual_cmd_s) > self.manual_cmd_timeout_s
        if stale:
            vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
        else:
            vx = self.manual_vx
            vy = self.manual_vy
            vz = self.manual_vz
            yaw_rate = self.manual_yaw_rate
        self._publish_enable(True)
        self._publish_twist(vx, vy, vz, yaw_rate, rate_limit=False)

    def _obstacle_repulsion_world_vector(self) -> Tuple[float, float]:
        if self.last_scan_ranges is None or self.last_scan_ranges.size == 0:
            return 0.0, 0.0
        
        # Calculate repulsion force from obstacles
        rep_x, rep_y = 0.0, 0.0
        dist_threshold = self.obstacle_avoid_dist_m
        
        # Filter for close obstacles
        close_idx = self.last_scan_ranges < dist_threshold
        close_ranges = self.last_scan_ranges[close_idx]
        close_angles = self.last_scan_angles[close_idx]
        
        if close_ranges.size == 0:
            return 0.0, 0.0
            
        # Inverse-square repulsion: push away from nearest obstacles
        weights = 1.0 / np.square(close_ranges + 0.05) # Add small epsilon to avoid div by zero
        # Sensor frame (forward is x, left is y)
        body_rep_x = -np.sum(weights * np.cos(close_angles))
        body_rep_y = -np.sum(weights * np.sin(close_angles))
        
        # Normalize and apply gain
        mag = math.hypot(body_rep_x, body_rep_y)
        if mag > 1e-6:
            body_rep_x = (body_rep_x / mag) * self.obstacle_repulse_gain
            body_rep_y = (body_rep_y / mag) * self.obstacle_repulse_gain
            
        # Rotate from body to world
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        world_rep_x = c * body_rep_x - s * body_rep_y
        world_rep_y = s * body_rep_x + c * body_rep_y
        return world_rep_x, world_rep_y

    def _goto_with_limits(
        self,
        x_ref: float,
        y_ref: float,
        z_ref: float,
        *,
        max_xy_speed: Optional[float] = None,
        max_z_speed: Optional[float] = None,
        xy_tol: float = 0.25,
        z_tol: float = 0.20,
        align_heading: bool = True,
    ) -> bool:
        now = self._now_s()
        
        # Auto-RTB on low battery
        if self.battery_percent < self.battery_low_warn_percent and self.phase == "MAPPING":
            self.get_logger().warn(f"Battery Low ({self.battery_percent:.1f}%). Aborting mission for safety.")
            self._set_phase("RETURN_HOME_AT_4M")
            return False

        ex = x_ref - self.x
        ey = y_ref - self.y
        ez = z_ref - self.z
        dist_xy = math.hypot(ex, ey)
        xy_speed_limit = self.max_xy_speed if max_xy_speed is None else max_xy_speed
        z_speed_limit = self.max_z_speed if max_z_speed is None else max_z_speed
        if self.phase == "MAPPING" and max_xy_speed is None:
            xy_speed_limit = self.mapping_speed_boost
            if dist_xy < self.mapping_precision_radius_m:
                alpha = self._clamp(dist_xy / max(self.mapping_precision_radius_m, 1e-6), 0.0, 1.0)
                xy_speed_limit = self.mapping_precision_min_speed + (
                    self.mapping_speed_boost - self.mapping_precision_min_speed
                ) * alpha

        nom_vx = self._clamp(0.85 * ex, -xy_speed_limit, xy_speed_limit)
        nom_vy = self._clamp(0.85 * ey, -xy_speed_limit, xy_speed_limit)
        
        # Repulsion vectors
        boundary_rep_vx, boundary_rep_vy = (0.0, 0.0)
        obstacle_rep_vx, obstacle_rep_vy = (0.0, 0.0)
        if self.phase == "MAPPING":
            if self._near_outer_boundary():
                boundary_rep_vx, boundary_rep_vy = self._down_repulsion_world_vector()
            obstacle_rep_vx, obstacle_rep_vy = self._obstacle_repulsion_world_vector()
            
        desired_vx = nom_vx + boundary_rep_vx + obstacle_rep_vx
        desired_vy = nom_vy + boundary_rep_vy + obstacle_rep_vy

        desired_speed = math.hypot(desired_vx, desired_vy)
        if desired_speed > xy_speed_limit:
            scale = xy_speed_limit / max(desired_speed, 1e-6)
            desired_vx *= scale
            desired_vy *= scale

        use_heading_lock = align_heading and (desired_speed > 0.02)
        vx, vy, yaw_rate = self._compose_xy_with_heading_lock(desired_vx, desired_vy, now, use_heading_lock, xy_speed_limit)
        vz = self._compute_vz(z_ref, z_speed_limit, now)
        z_err_for_ok = ez
        if self._tof_is_valid(now) and (self.tof_range_m < self.tof_full_weight_range_m):
            z_err_for_ok = z_ref - self.tof_range_m

        self._publish_twist(vx, vy, vz, yaw_rate)

        xy_ok = dist_xy < xy_tol
        z_ok = abs(z_err_for_ok) < z_tol
        return xy_ok and z_ok

    def _goto(self, x_ref: float, y_ref: float, z_ref: float) -> bool:
        return self._goto_with_limits(x_ref, y_ref, z_ref)

    def _goto_with_yaw_lock(
        self,
        x_ref: float,
        y_ref: float,
        z_ref: float,
        target_yaw: float,
        *,
        max_xy_speed: Optional[float] = None,
        max_z_speed: Optional[float] = None,
        xy_tol: float = 0.25,
        z_tol: float = 0.20,
    ) -> Tuple[bool, bool]:
        now = self._now_s()
        ex = x_ref - self.x
        ey = y_ref - self.y
        dist_xy = math.hypot(ex, ey)
        xy_speed_limit = self.max_xy_speed if max_xy_speed is None else max_xy_speed
        z_speed_limit = self.max_z_speed if max_z_speed is None else max_z_speed

        nom_vx = self._clamp(0.85 * ex, -xy_speed_limit, xy_speed_limit)
        nom_vy = self._clamp(0.85 * ey, -xy_speed_limit, xy_speed_limit)

        desired_vx = nom_vx
        desired_vy = nom_vy
        desired_speed = math.hypot(desired_vx, desired_vy)
        if desired_speed > xy_speed_limit:
            scale = xy_speed_limit / max(desired_speed, 1e-6)
            desired_vx *= scale
            desired_vy *= scale

        cmd_vx, cmd_vy = desired_vx, desired_vy
        if self.cmd_vel_is_body_frame:
            cmd_vx, cmd_vy = self._world_to_body_xy(desired_vx, desired_vy)

        yaw_ready, yaw_rate = self._heading_alignment_gate(target_yaw, now)
        vz = self._compute_vz(z_ref, z_speed_limit, now)
        z_err_for_ok = z_ref - self.z
        if self._tof_is_valid(now) and (self.tof_range_m < self.tof_full_weight_range_m):
            z_err_for_ok = z_ref - self.tof_range_m

        self._publish_twist(cmd_vx, cmd_vy, vz, yaw_rate)

        xy_ok = dist_xy < xy_tol
        z_ok = abs(z_err_for_ok) < z_tol
        return xy_ok and z_ok, yaw_ready

    def _yellow_recent(self) -> Tuple[bool, str]:
        now = self._now_s()
        front_recent = (now - self.front_seen_time) < self.yellow_hold_s
        down_recent = (now - self.down_seen_time) < self.yellow_hold_s

        if front_recent and down_recent:
            return True, "both"
        if front_recent:
            return True, "front"
        if down_recent:
            return True, "down"
        return False, ""

    def _run_boundary_avoidance(self, now: float):
        desired_vx, desired_vy = self._boundary_retreat_vector_world()
        target_yaw = math.atan2(desired_vy, desired_vx)
        yaw_err = self._angle_wrap(target_yaw - self.yaw)
        yaw_rate = self._clamp(
            self.auto_yaw_kp * yaw_err,
            -self.auto_max_yaw_rate,
            self.auto_max_yaw_rate,
        )
        vz = self._compute_vz(self.target_alt_m, 0.25, now)

        if now < self.boundary_hover_until:
            self._publish_twist(0.0, 0.0, vz, yaw_rate)
            return

        if abs(yaw_err) > self.boundary_move_yaw_tolerance_rad:
            self._publish_twist(0.0, 0.0, vz, yaw_rate)
            return

        if self.cmd_vel_is_body_frame:
            vx_body, vy_body = self._world_to_body_xy(desired_vx, desired_vy)
            # Never force forward motion when retreat vector is still behind body-x.
            # Hold and keep yawing until the inward vector is actually in front.
            if vx_body <= 0.0:
                self._publish_twist(0.0, 0.0, vz, yaw_rate)
                return
            cmd_vx = self._clamp(vx_body, self.boundary_min_escape_speed, self.max_xy_speed)
            cmd_vy = self._clamp(vy_body, -self.boundary_max_lateral_speed, self.boundary_max_lateral_speed)
            self._publish_twist(cmd_vx, cmd_vy, vz, yaw_rate)
            return

        dir_x, dir_y, dir_mag = self._normalize_xy(desired_vx, desired_vy)
        speed = max(self.boundary_min_escape_speed, math.hypot(desired_vx, desired_vy))
        if dir_mag < 1e-6:
            self._publish_twist(0.0, 0.0, vz, yaw_rate)
        else:
            self._publish_twist(speed * dir_x, speed * dir_y, vz, yaw_rate)

    def _near_outer_boundary(self) -> bool:
        return abs(self.x) >= self.boundary_trigger_extent_m or abs(self.y) >= self.boundary_trigger_extent_m

    def _distance_from_base(self) -> float:
        return math.hypot(self.x - self.base_xy[0], self.y - self.base_xy[1])

    def _speed_mps(self) -> float:
        return math.sqrt(self.vx * self.vx + self.vy * self.vy + self.vz * self.vz)

    def _mapping_is_stuck(self, now_s: float) -> bool:
        if self.phase != "MAPPING":
            self.mapping_stuck_since = None
            return False
        if self.waypoint_idx >= len(self.waypoints):
            self.mapping_stuck_since = None
            return False

        tx, ty, _ = self.waypoints[self.waypoint_idx]
        dist_xy = math.hypot(tx - self.x, ty - self.y)
        speed = self._speed_mps()
        if dist_xy < self.mapping_stuck_distance_m or speed > self.mapping_stuck_speed_mps:
            self.mapping_stuck_since = None
            return False

        if self.mapping_stuck_since is None:
            self.mapping_stuck_since = now_s
            return False
        return (now_s - self.mapping_stuck_since) >= self.mapping_stuck_timeout_s

    def _update_battery_estimate(self, now_s: float):
        dt = max(0.0, now_s - self.last_battery_update_s)
        self.last_battery_update_s = now_s
        if dt <= 0.0:
            return

        raw_speed = self._speed_mps()
        if not math.isfinite(raw_speed):
            raw_speed = 0.0
        raw_speed = self._clamp(raw_speed, 0.0, self.max_reasonable_speed_mps)

        raw_dist = self._distance_from_base()
        if not math.isfinite(raw_dist):
            raw_dist = 0.0
        raw_dist = self._clamp(raw_dist, 0.0, self.max_reasonable_distance_m)

        a = self._clamp(self.dashboard_smoothing_alpha, 0.01, 1.0)
        self.dashboard_speed_mps = (1.0 - a) * self.dashboard_speed_mps + a * raw_speed
        self.dashboard_distance_m = (1.0 - a) * self.dashboard_distance_m + a * raw_dist

        speed_load = min(1.0, self.dashboard_speed_mps / max(self.mapping_speed_boost, 1e-6))
        dyn_load = max(self.motor_load, speed_load)
        if not self.has_odom:
            dyn_load = 0.0
        drain_rate = self.battery_idle_drain_pct_per_s + (
            self.battery_full_drain_pct_per_s - self.battery_idle_drain_pct_per_s
        ) * dyn_load
        self.battery_percent = self._clamp(self.battery_percent - drain_rate * dt, 0.0, 100.0)

    def _dashboard_image(self) -> np.ndarray:
        h, w = 230, 420
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = (24, 24, 24)

        speed = self.dashboard_speed_mps
        dist = self.dashboard_distance_m
        progress = 0.0
        if len(self.waypoints) > 0:
            progress = 100.0 * float(self.waypoint_idx) / float(len(self.waypoints))
            progress = self._clamp(progress, 0.0, 100.0)
        mapped = self._clamp(self.map_known_percent, 0.0, 100.0)
        covered = self._clamp(self.arena_coverage_percent, 0.0, 100.0)

        batt_color = (70, 200, 70)
        if self.battery_percent <= self.battery_low_warn_percent:
            batt_color = (0, 215, 255)
        if self.battery_percent <= self.battery_critical_percent:
            batt_color = (40, 40, 230)

        cv2.putText(img, "Mission Dashboard", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(
            img,
            f"Mode: {'MANUAL' if self.manual_mode_enabled else 'AUTO'}   Phase: {self.phase}",
            (12, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.47,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(img, f"Speed: {speed:.2f} m/s", (12, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(
            img,
            f"Distance from base: {dist:.2f} m",
            (12, 104),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            img,
            f"Mapped (2D): {mapped:.1f}%",
            (12, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            img,
            f"Arena covered: {covered:.1f}%",
            (12, 156),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            img,
            f"Sweep progress: {progress:.1f}%",
            (12, 182),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        bar_x, bar_y, bar_w, bar_h = 12, 198, 280, 24
        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (85, 85, 85), 2)
        fill_w = int((self.battery_percent / 100.0) * (bar_w - 4))
        fill_w = max(0, min(bar_w - 4, fill_w))
        if fill_w > 0:
            cv2.rectangle(img, (bar_x + 2, bar_y + 2), (bar_x + 2 + fill_w, bar_y + bar_h - 2), batt_color, -1)
        cv2.putText(
            img,
            f"Battery: {self.battery_percent:.1f}%",
            (bar_x + bar_w + 12, bar_y + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            batt_color,
            2,
            cv2.LINE_AA,
        )
        return img

    def _maybe_trigger_failsafe(self):
        if self.failsafe_triggered or not self.failsafe_test_enabled:
            return
        if self.phase != "MAPPING":
            return

        dist = self._distance_from_base()
        if dist < self.failsafe_trigger_distance_m:
            return

        self.failsafe_triggered = True
        self.boundary_avoid_until = -1.0
        self.get_logger().warn(
            f"FAILSAFE: motor {self.failed_motor_index + 1} output cut by ESC relay at {dist:.2f} m from base. "
            "Propeller stopped, switching to 3-motor degraded return and safe landing."
        )
        self._set_phase("FAILSAFE_RETURN_HOME_AT_4M")

    def _publish_scan_tf(self):
        # The LiDAR is rigidly mounted above base_link in model.sdf.
        if self.scan_frame_id == "base_link":
            return
        if self.published_static_scan_frame_id == self.scan_frame_id:
            return
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = self.scan_frame_id
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.065
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf_msg)
        self.published_static_scan_frame_id = self.scan_frame_id

    def _maybe_show_windows(self):
        if self.show_windows:
            if self.front_debug is not None:
                cv2.imshow("Front Camera - Yellow Boundary", self.front_debug)
            if self.down_debug is not None:
                cv2.imshow("Belly Camera - Yellow Boundary", self.down_debug)
            cv2.imshow("Mission Dashboard", self._dashboard_image())
            cv2.waitKey(1)

        now = self._now_s()
        if now - self.last_dashboard_log_s > 1.0:
            self.last_dashboard_log_s = now
            self.get_logger().info(
                f"[Dashboard] battery={self.battery_percent:.1f}% speed={self.dashboard_speed_mps:.2f}m/s "
                f"dist_base={self.dashboard_distance_m:.2f}m mapped={self.map_known_percent:.1f}% "
                f"covered={self.arena_coverage_percent:.1f}% phase={self.phase}"
            )

    def _set_phase(self, phase: str):
        self.phase = phase
        self.phase_start = self._now_s()
        self.reached_since = None
        self.active_waypoint_start_s = None
        self.heading_aligned_since = None
        self.heading_align_wait_start_s = None
        self.mapping_stuck_since = None
        if phase != "MAPPING":
            self.boundary_avoid_start_s = -1.0
        self.get_logger().info(f"Phase -> {phase}")

    def _control_tick(self):
        now = self._now_s()
        self._update_battery_estimate(now)
        self._update_coverage(now)
        self._maybe_show_windows()
        # Keep lidar -> base_link TF available during startup so slam_toolbox
        # message filters don't backlog waiting for this transform.
        self._publish_scan_tf()
        if self.rtb_requested and (not self.failsafe_triggered):
            self.rtb_requested = False
            return_phases = {
                "RETURN_HOME_AT_4M",
                "HOVER_HOME_AT_4M",
                "ALIGN_YAW_AT_4M",
                "DESCEND_TO_2M",
                "HOLD_AT_2M",
                "ALIGN_YAW_AT_2M",
                "LAND",
                "DONE",
            }
            if self.phase not in return_phases:
                self.get_logger().warn("RTB override: aborting current phase and returning to base.")
                self._set_phase("RETURN_HOME_AT_4M")
        if self.manual_mode_enabled:
            self._run_manual_override(now)
            return

        if not (self.has_odom and self.has_scan and self.has_front and self.has_down):
            self._publish_enable(False)
            self._publish_twist(0.0, 0.0, 0.0)
            return

        if self.phase == "DONE":
            self._publish_enable(False)
            self._publish_twist(0.0, 0.0, 0.0)
            return

        self._publish_enable(True)
        if self.failsafe_test_enabled and (not self.has_motor_cmd) and (now - self.last_motor_cmd_warn_s > 3.0):
            self.get_logger().warn("Motor relay has not received /motor_speed_cmd_in yet; waiting for first motor command.")
            self.last_motor_cmd_warn_s = now
        self._maybe_trigger_failsafe()

        boundary_seen, source = self._yellow_recent()
        boundary_risk = self.down_repulse_strength
        if self.phase == "MAPPING" and (not self.failsafe_triggered):
            near_boundary = self._near_outer_boundary()
            guard_trigger = near_boundary and (
                (boundary_risk >= self.boundary_guard_enter_strength) or boundary_seen
            )
            if guard_trigger:
                if now >= self.boundary_avoid_until:
                    self.boundary_hover_until = now + self.boundary_hover_s
                    self.boundary_avoid_start_s = now
                    self.heading_aligned_since = None
                    self.heading_align_wait_start_s = None
                if self.boundary_avoid_start_s < 0.0:
                    self.boundary_avoid_start_s = now
                guard_extension = self.avoid_duration_s + boundary_risk
                avoid_deadline = self.boundary_avoid_start_s + self.boundary_avoid_max_duration_s
                self.boundary_avoid_until = min(
                    max(self.boundary_avoid_until, now + guard_extension),
                    avoid_deadline,
                )
                if now - self.last_boundary_log_s > 1.0:
                    self.get_logger().info(
                        "Boundary guard trigger: "
                        f"source={source or 'down-repulse'} front={self.front_yellow_ratio:.3f} "
                        f"down={self.down_yellow_ratio:.3f} repel={self.down_repulse_strength:.2f}"
                    )
                    self.last_boundary_log_s = now
            elif near_boundary and (now < self.boundary_avoid_until) and (boundary_risk > self.boundary_guard_exit_strength):
                if self.boundary_avoid_start_s < 0.0:
                    self.boundary_avoid_start_s = now
                avoid_deadline = self.boundary_avoid_start_s + self.boundary_avoid_max_duration_s
                self.boundary_avoid_until = min(max(self.boundary_avoid_until, now + 0.35), avoid_deadline)

        if self.phase == "WAIT_SENSORS":
            self._set_phase("TAKEOFF")
            return

        if self.phase == "TAKEOFF":
            reached = self._goto(self.base_xy[0], self.base_xy[1], self.target_alt_m)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 1.5:
                    self._set_phase("MAPPING")
                    self.get_logger().info("Reached 4.0 m. Starting corner-to-corner boundary sweep mapping with lidar + cameras.")
            else:
                self.reached_since = None
            return

        if self.phase == "MAPPING":
            if self._mapping_is_stuck(now):
                self.get_logger().warn(
                    f"Waypoint {self.waypoint_idx + 1}/{len(self.waypoints)} appears stuck; advancing and clearing boundary avoidance."
                )
                self.waypoint_idx += 1
                self.reached_since = None
                self.active_waypoint_start_s = None
                self.heading_aligned_since = None
                self.heading_align_wait_start_s = None
                self.boundary_avoid_until = -1.0
                self.boundary_hover_until = -1.0
                self.boundary_avoid_start_s = -1.0
                self.mapping_stuck_since = None
                return

            if now < self.boundary_avoid_until:
                if self.boundary_avoid_start_s > 0.0 and (now - self.boundary_avoid_start_s) > self.boundary_avoid_max_duration_s:
                    self.get_logger().warn("Boundary avoidance max duration reached; resuming waypoint tracking.")
                    self.boundary_avoid_until = -1.0
                    self.boundary_hover_until = -1.0
                    self.boundary_avoid_start_s = -1.0
                    self.heading_aligned_since = None
                    self.heading_align_wait_start_s = None
                else:
                    self._run_boundary_avoidance(now)
                    return
            else:
                self.boundary_avoid_start_s = -1.0

            if self.waypoint_idx >= len(self.waypoints):
                self._set_phase("RETURN_HOME_AT_4M")
                return

            if self.active_waypoint_start_s is None:
                self.active_waypoint_start_s = now
            elif now - self.active_waypoint_start_s > self.waypoint_timeout_s:
                self.get_logger().warn(
                    f"Waypoint {self.waypoint_idx + 1}/{len(self.waypoints)} timeout; advancing to keep boundary sweep coverage."
                )
                self.waypoint_idx += 1
                self.reached_since = None
                self.active_waypoint_start_s = None
                self.heading_aligned_since = None
                self.heading_align_wait_start_s = None
                return

            tx, ty, tz = self.waypoints[self.waypoint_idx]
            reached = self._goto(tx, ty, tz)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.15:
                    self.waypoint_idx += 1
                    self.reached_since = None
                    self.active_waypoint_start_s = None
                    self.heading_aligned_since = None
                    self.heading_align_wait_start_s = None
                    self.get_logger().info(
                        f"Sweep waypoint {self.waypoint_idx}/{len(self.waypoints)} complete"
                    )
            else:
                self.reached_since = None
            return

        if self.phase == "RETURN_HOME_AT_4M":
            reached = self._goto(self.base_xy[0], self.base_xy[1], self.target_alt_m)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.8:
                    self._set_phase("HOVER_HOME_AT_4M")
            else:
                self.reached_since = None
            return

        if self.phase == "FAILSAFE_RETURN_HOME_AT_4M":
            reached = self._goto_with_limits(
                self.base_xy[0],
                self.base_xy[1],
                self.target_alt_m,
                max_xy_speed=self.failsafe_three_motor_xy_speed,
                max_z_speed=self.failsafe_three_motor_z_speed,
                xy_tol=0.20,
                z_tol=0.15,
            )
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.8:
                    self._set_phase("FAILSAFE_HOVER_HOME_AT_4M")
            else:
                self.reached_since = None
            return

        if self.phase == "FAILSAFE_HOVER_HOME_AT_4M":
            self._goto_with_limits(
                self.base_xy[0],
                self.base_xy[1],
                self.target_alt_m,
                max_xy_speed=self.failsafe_three_motor_xy_speed,
                max_z_speed=self.failsafe_three_motor_z_speed,
                xy_tol=0.20,
                z_tol=0.15,
            )
            if now - self.phase_start > self.home_hover_s:
                self._set_phase("FAILSAFE_ALIGN_YAW_AT_4M")
            return

        if self.phase == "FAILSAFE_ALIGN_YAW_AT_4M":
            pos_ok, yaw_ok = self._goto_with_yaw_lock(
                self.base_xy[0],
                self.base_xy[1],
                self.target_alt_m,
                self.landing_yaw_rad,
                max_xy_speed=self.failsafe_three_motor_xy_speed,
                max_z_speed=self.failsafe_three_motor_z_speed,
                xy_tol=0.20,
                z_tol=0.15,
            )
            if pos_ok and yaw_ok:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.6:
                    self._set_phase("FAILSAFE_DESCEND_TO_2M")
            else:
                self.reached_since = None
            return

        if self.phase == "FAILSAFE_DESCEND_TO_2M":
            reached = self._goto_with_limits(
                self.base_xy[0],
                self.base_xy[1],
                self.mid_alt_m,
                max_xy_speed=self.failsafe_three_motor_xy_speed,
                max_z_speed=self.failsafe_three_motor_z_speed,
                xy_tol=0.20,
                z_tol=0.12,
            )
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.8:
                    self._set_phase("FAILSAFE_HOLD_AT_2M")
            else:
                self.reached_since = None
            return

        if self.phase == "FAILSAFE_HOLD_AT_2M":
            self._goto_with_limits(
                self.base_xy[0],
                self.base_xy[1],
                self.mid_alt_m,
                max_xy_speed=self.failsafe_three_motor_xy_speed,
                max_z_speed=self.failsafe_three_motor_z_speed,
                xy_tol=0.20,
                z_tol=0.12,
            )
            if now - self.phase_start > self.mid_alt_hover_s:
                self._set_phase("FAILSAFE_ALIGN_YAW_AT_2M")
            return

        if self.phase == "FAILSAFE_ALIGN_YAW_AT_2M":
            pos_ok, yaw_ok = self._goto_with_yaw_lock(
                self.base_xy[0],
                self.base_xy[1],
                self.mid_alt_m,
                self.landing_yaw_rad,
                max_xy_speed=self.failsafe_three_motor_xy_speed,
                max_z_speed=self.failsafe_three_motor_z_speed,
                xy_tol=0.18,
                z_tol=0.12,
            )
            if pos_ok and yaw_ok:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.6:
                    self._set_phase("FAILSAFE_LAND")
            else:
                self.reached_since = None
            return

        if self.phase == "FAILSAFE_LAND":
            pos_ok, yaw_ok = self._goto_with_yaw_lock(
                self.base_xy[0],
                self.base_xy[1],
                self.landing_z_m,
                self.landing_yaw_rad,
                max_xy_speed=0.25,
                max_z_speed=self.failsafe_three_motor_land_z_speed,
                xy_tol=0.18,
                z_tol=0.07,
            )
            if pos_ok and yaw_ok:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 1.0:
                    self._set_phase("DONE")
            else:
                self.reached_since = None
            return

        if self.phase == "HOVER_HOME_AT_4M":
            self._goto(self.base_xy[0], self.base_xy[1], self.target_alt_m)
            if now - self.phase_start > self.home_hover_s:
                self._set_phase("ALIGN_YAW_AT_4M")
            return

        if self.phase == "ALIGN_YAW_AT_4M":
            pos_ok, yaw_ok = self._goto_with_yaw_lock(
                self.base_xy[0],
                self.base_xy[1],
                self.target_alt_m,
                self.landing_yaw_rad,
                max_xy_speed=0.40,
                max_z_speed=self.slow_descend_max_z_speed,
                xy_tol=0.18,
                z_tol=0.12,
            )
            if pos_ok and yaw_ok:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.6:
                    self._set_phase("DESCEND_TO_2M")
            else:
                self.reached_since = None
            return

        if self.phase == "DESCEND_TO_2M":
            reached = self._goto_with_limits(
                self.base_xy[0],
                self.base_xy[1],
                self.mid_alt_m,
                max_xy_speed=0.55,
                max_z_speed=self.slow_descend_max_z_speed,
                z_tol=0.12,
            )
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.8:
                    self._set_phase("HOLD_AT_2M")
            else:
                self.reached_since = None
            return

        if self.phase == "HOLD_AT_2M":
            self._goto_with_limits(
                self.base_xy[0],
                self.base_xy[1],
                self.mid_alt_m,
                max_xy_speed=0.50,
                max_z_speed=self.slow_descend_max_z_speed,
                z_tol=0.12,
            )
            if now - self.phase_start > self.mid_alt_hover_s:
                self._set_phase("ALIGN_YAW_AT_2M")
            return

        if self.phase == "ALIGN_YAW_AT_2M":
            pos_ok, yaw_ok = self._goto_with_yaw_lock(
                self.base_xy[0],
                self.base_xy[1],
                self.mid_alt_m,
                self.landing_yaw_rad,
                max_xy_speed=0.35,
                max_z_speed=self.slow_descend_max_z_speed,
                xy_tol=0.16,
                z_tol=0.10,
            )
            if pos_ok and yaw_ok:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.6:
                    self._set_phase("LAND")
            else:
                self.reached_since = None
            return

        if self.phase == "LAND":
            pos_ok, yaw_ok = self._goto_with_yaw_lock(
                self.base_xy[0],
                self.base_xy[1],
                self.landing_z_m,
                self.landing_yaw_rad,
                max_xy_speed=0.35,
                max_z_speed=self.landing_max_z_speed,
                xy_tol=0.20,
                z_tol=0.07,
            )
            if pos_ok and yaw_ok:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 1.0:
                    self._set_phase("DONE")
            else:
                self.reached_since = None
            return

    def destroy_node(self):
        self._publish_twist(0.0, 0.0, 0.0)
        self._publish_enable(False)
        if self.show_windows:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = SlamBoundaryMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
