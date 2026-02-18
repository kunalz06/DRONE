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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class SlamBoundaryMission(Node):
    def __init__(self):
        super().__init__("slam_boundary_mapping_mission")

        self.target_alt_m = 4.0
        self.control_hz = 20.0
        self.base_xy = (0.0, 0.0)
        self.max_xy_speed = 1.1
        self.max_z_speed = 0.6
        self.arena_half_extent_m = 8.0
        self.lawnmower_edge_margin_m = 1.2
        self.lawnmower_lane_spacing_m = 1.8
        self.boundary_trigger_extent_m = 7.2
        self.waypoint_timeout_s = 22.0
        self.home_hover_s = 2.0
        self.mid_alt_m = 2.0
        self.mid_alt_hover_s = 2.0
        self.landing_z_m = 0.20
        self.slow_descend_max_z_speed = 0.16
        self.landing_max_z_speed = 0.12
        self.failsafe_test_enabled = False
        self.failsafe_trigger_distance_m = 2.9
        self.failed_motor_index = 3  # 0-based: rotor_3 (4th motor)
        self.failsafe_three_motor_xy_speed = 0.45
        self.failsafe_three_motor_z_speed = 0.20
        self.failsafe_three_motor_land_z_speed = 0.08
        self.esc_current_limit_a = 40.0
        self.manual_cmd_timeout_s = 0.6
        self.manual_max_xy_speed = 1.2
        self.manual_max_z_speed = 0.7
        self.manual_max_yaw_rate = 1.2

        self.front_yellow_threshold = 0.010
        self.down_yellow_threshold = 0.010
        self.yellow_hold_s = 0.5
        self.avoid_duration_s = 1.2

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pub_enable = self.create_publisher(Bool, "/drone_enable", 10)
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_motor_cmd = self.create_publisher(Actuators, "/motor_speed_cmd_out", 10)

        self.sub_odom = self.create_subscription(Odometry, "/odom", self._odom_cb, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.sub_front = self.create_subscription(Image, "/front_camera/image_raw", self._front_cb, qos_profile_sensor_data)
        self.sub_down = self.create_subscription(Image, "/down_camera/image_raw", self._down_cb, qos_profile_sensor_data)
        self.sub_motor_cmd = self.create_subscription(
            Actuators, "/motor_speed_cmd_in", self._motor_cmd_cb, qos_profile_sensor_data
        )
        self.sub_manual_mode = self.create_subscription(Bool, "/manual_mode", self._manual_mode_cb, 10)
        self.sub_manual_cmd = self.create_subscription(Twist, "/manual_cmd_vel", self._manual_cmd_cb, 10)

        self.has_odom = False
        self.has_scan = False
        self.has_front = False
        self.has_down = False
        self.has_motor_cmd = False
        self.scan_frame_id = "lidar_link"

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.front_yellow_ratio = 0.0
        self.down_yellow_ratio = 0.0
        self.front_seen_time = -1.0
        self.down_seen_time = -1.0

        self.show_windows = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        self.front_debug = None
        self.down_debug = None

        self.boundary_avoid_until = -1.0
        self.last_boundary_log_s = -1.0

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

        self.waypoints = self._build_mapping_waypoints()
        self.waypoint_idx = 0
        self.active_waypoint_start_s = None

        self.control_timer = self.create_timer(1.0 / self.control_hz, self._control_tick)

        self.get_logger().info("SLAM mapping mission node started.")
        self.get_logger().info(
            f"Lawnmower coverage ready with {len(self.waypoints)} waypoints (lane spacing {self.lawnmower_lane_spacing_m:.2f} m)."
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

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _build_mapping_waypoints(self) -> List[Tuple[float, float, float]]:
        z = self.target_alt_m
        x_min = -self.arena_half_extent_m + self.lawnmower_edge_margin_m
        x_max = self.arena_half_extent_m - self.lawnmower_edge_margin_m
        y_min = -self.arena_half_extent_m + self.lawnmower_edge_margin_m
        y_max = self.arena_half_extent_m - self.lawnmower_edge_margin_m
        span_x = max(0.1, x_max - x_min)
        lane_count = max(2, int(math.ceil(span_x / self.lawnmower_lane_spacing_m)) + 1)
        x_step = span_x / float(lane_count - 1)

        waypoints: List[Tuple[float, float, float]] = []
        for lane_idx in range(lane_count):
            x_lane = x_min + lane_idx * x_step
            if lane_idx % 2 == 0:
                waypoints.append((x_lane, y_min, z))
                waypoints.append((x_lane, y_max, z))
            else:
                waypoints.append((x_lane, y_max, z))
                waypoints.append((x_lane, y_min, z))
        return waypoints

    def _odom_cb(self, msg: Odometry):
        self.has_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def _scan_cb(self, _msg: LaserScan):
        self.has_scan = True
        if _msg.header.frame_id:
            self.scan_frame_id = _msg.header.frame_id

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
        if not self.manual_mode_enabled:
            self.manual_vx = 0.0
            self.manual_vy = 0.0
            self.manual_vz = 0.0
            self.manual_yaw_rate = 0.0

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
        ratio, vis = self._yellow_ratio_and_vis(msg)
        self.has_down = True
        self.down_yellow_ratio = ratio
        if ratio > self.down_yellow_threshold:
            self.down_seen_time = self._now_s()
        self.down_debug = vis

    def _yellow_ratio_and_vis(self, msg: Image) -> Tuple[float, Optional[np.ndarray]]:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"Image decode failed: {exc}")
            return 0.0, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([18, 80, 80], dtype=np.uint8)
        upper = np.array([38, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

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

    @staticmethod
    def _clamp(val: float, low: float, high: float) -> float:
        return max(low, min(high, val))

    def _publish_enable(self, enabled: bool):
        if not rclpy.ok():
            return
        msg = Bool()
        msg.data = enabled
        try:
            self.pub_enable.publish(msg)
        except Exception:
            pass

    def _publish_twist(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        if not rclpy.ok():
            return
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
        self._publish_twist(vx, vy, vz, yaw_rate)

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
    ) -> bool:
        ex = x_ref - self.x
        ey = y_ref - self.y
        ez = z_ref - self.z
        dist_xy = math.hypot(ex, ey)
        xy_speed_limit = self.max_xy_speed if max_xy_speed is None else max_xy_speed
        z_speed_limit = self.max_z_speed if max_z_speed is None else max_z_speed

        vx = self._clamp(0.85 * ex, -xy_speed_limit, xy_speed_limit)
        vy = self._clamp(0.85 * ey, -xy_speed_limit, xy_speed_limit)
        vz = self._clamp(0.9 * ez, -z_speed_limit, z_speed_limit)

        self._publish_twist(vx, vy, vz)

        xy_ok = dist_xy < xy_tol
        z_ok = abs(ez) < z_tol
        return xy_ok and z_ok

    def _goto(self, x_ref: float, y_ref: float, z_ref: float) -> bool:
        return self._goto_with_limits(x_ref, y_ref, z_ref)

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

    def _run_boundary_avoidance(self):
        dx = self.base_xy[0] - self.x
        dy = self.base_xy[1] - self.y
        norm = math.hypot(dx, dy)

        if norm < 1e-3:
            vx, vy = 0.0, 0.0
        else:
            inward_speed = 0.9
            vx = inward_speed * (dx / norm)
            vy = inward_speed * (dy / norm)

        vz = self._clamp(0.8 * (self.target_alt_m - self.z), -0.25, 0.25)
        self._publish_twist(vx, vy, vz)

    def _near_outer_boundary(self) -> bool:
        return abs(self.x) >= self.boundary_trigger_extent_m or abs(self.y) >= self.boundary_trigger_extent_m

    def _distance_from_base(self) -> float:
        return math.hypot(self.x - self.base_xy[0], self.y - self.base_xy[1])

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
        self.tf_broadcaster.sendTransform(tf_msg)

    def _maybe_show_windows(self):
        if not self.show_windows:
            return
        if self.front_debug is not None:
            cv2.imshow("Front Camera - Yellow Boundary", self.front_debug)
        if self.down_debug is not None:
            cv2.imshow("Belly Camera - Yellow Boundary", self.down_debug)
        cv2.waitKey(1)

    def _set_phase(self, phase: str):
        self.phase = phase
        self.phase_start = self._now_s()
        self.reached_since = None
        self.active_waypoint_start_s = None
        self.get_logger().info(f"Phase -> {phase}")

    def _control_tick(self):
        self._maybe_show_windows()

        now = self._now_s()
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
        self._publish_scan_tf()
        if not self.has_motor_cmd and (now - self.last_motor_cmd_warn_s > 3.0):
            self.get_logger().warn("Motor relay has not received /motor_speed_cmd_in yet; waiting for first motor command.")
            self.last_motor_cmd_warn_s = now
        self._maybe_trigger_failsafe()

        boundary_seen, source = self._yellow_recent()
        if (not self.failsafe_triggered) and boundary_seen and self._near_outer_boundary():
            self.boundary_avoid_until = max(self.boundary_avoid_until, now + self.avoid_duration_s)
            if now - self.last_boundary_log_s > 1.0:
                self.get_logger().info(
                    f"Boundary detected from {source} camera(s): front={self.front_yellow_ratio:.3f}, down={self.down_yellow_ratio:.3f}"
                )
                self.last_boundary_log_s = now

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
                    self.get_logger().info("Reached 4.0 m. Starting arena mapping with lidar + cameras.")
            else:
                self.reached_since = None
            return

        if self.phase == "MAPPING":
            if now < self.boundary_avoid_until:
                self._run_boundary_avoidance()
                return

            if self.waypoint_idx >= len(self.waypoints):
                self._set_phase("RETURN_HOME_AT_4M")
                return

            if self.active_waypoint_start_s is None:
                self.active_waypoint_start_s = now
            elif now - self.active_waypoint_start_s > self.waypoint_timeout_s:
                self.get_logger().warn(
                    f"Waypoint {self.waypoint_idx + 1}/{len(self.waypoints)} timeout; advancing to keep lawnmower coverage."
                )
                self.waypoint_idx += 1
                self.reached_since = None
                self.active_waypoint_start_s = None
                return

            tx, ty, tz = self.waypoints[self.waypoint_idx]
            reached = self._goto(tx, ty, tz)
            if reached:
                if self.reached_since is None:
                    self.reached_since = now
                elif now - self.reached_since > 0.8:
                    self.waypoint_idx += 1
                    self.reached_since = None
                    self.active_waypoint_start_s = None
                    self.get_logger().info(
                        f"Lawnmower waypoint {self.waypoint_idx}/{len(self.waypoints)} complete"
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
                self._set_phase("FAILSAFE_DESCEND_TO_2M")
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
                self._set_phase("FAILSAFE_LAND")
            return

        if self.phase == "FAILSAFE_LAND":
            reached = self._goto_with_limits(
                self.base_xy[0],
                self.base_xy[1],
                self.landing_z_m,
                max_xy_speed=0.25,
                max_z_speed=self.failsafe_three_motor_land_z_speed,
                xy_tol=0.18,
                z_tol=0.07,
            )
            if reached:
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
                self._set_phase("DESCEND_TO_2M")
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
                self._set_phase("LAND")
            return

        if self.phase == "LAND":
            reached = self._goto_with_limits(
                self.base_xy[0],
                self.base_xy[1],
                self.landing_z_m,
                max_xy_speed=0.35,
                max_z_speed=self.landing_max_z_speed,
                z_tol=0.07,
            )
            if reached:
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
