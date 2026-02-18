#!/usr/bin/env python3
import math
import os
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
import gz.msgs10.image_pb2 as image_pb2
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node
import cv2
import numpy as np


MODEL_NAME = "px4_x500_lidar_rgb"
TAKEOFF_ALT_M = 2.5
HIGH_ALT_M = 4.0
MID_ALT_M = 2.0
FIRST_CIRCLE_RADIUS_M = 1.0
SECOND_CIRCLE_RADIUS_M = 2.0
THIRD_CIRCLE_RADIUS_M = 2.0
CIRCLE_ANGULAR_SPEED_RAD_S = 0.45
BASE_HOVER_BEFORE_LAND_S = 2.0
BASE_X_M = 0.0
BASE_Y_M = 0.0
LAND_SETTLE_Z_M = 0.19
CONTROL_HZ = 20.0


class FlightState:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.has_odom = False

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.has_odom = True


class CameraViewer:
    def __init__(self, node: Node, topic: str):
        self.enabled = False
        self._frame = None
        self._lock = threading.Lock()
        self._warned_unsupported_format = False
        self._window_name = "Belly Camera Feed"

        # If there is no display server, skip rendering but keep mission running.
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            print("[WARN] No display server detected. Belly camera window disabled.")
            return

        ok = node.subscribe(Image, topic, self._image_cb)
        if not ok:
            print(f"[WARN] Failed to subscribe to camera topic: {topic}")
            return

        self.enabled = True
        print(f"[INFO] Belly camera viewer enabled on topic: {topic}")

    def _image_cb(self, msg: Image):
        frame = self._decode_image(msg)
        if frame is None:
            return
        with self._lock:
            self._frame = frame

    def _decode_image(self, msg: Image):
        fmt = msg.pixel_format_type
        width = msg.width
        height = msg.height
        if width <= 0 or height <= 0:
            return None

        if fmt == image_pb2.RGB_INT8:
            channels = 3
            cvt = cv2.COLOR_RGB2BGR
        elif fmt == image_pb2.RGBA_INT8:
            channels = 4
            cvt = cv2.COLOR_RGBA2BGR
        elif fmt == image_pb2.BGR_INT8:
            channels = 3
            cvt = None
        elif fmt == image_pb2.BGRA_INT8:
            channels = 4
            cvt = cv2.COLOR_BGRA2BGR
        else:
            if not self._warned_unsupported_format:
                print(f"[WARN] Unsupported camera pixel format: {fmt}")
                self._warned_unsupported_format = True
            return None

        expected_step = width * channels
        row_step = msg.step if msg.step > 0 else expected_step
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        needed = row_step * height
        if raw.size < needed:
            return None

        rows = raw[:needed].reshape((height, row_step))
        packed = rows[:, :expected_step]
        img = packed.reshape((height, width, channels))
        if cvt is not None:
            img = cv2.cvtColor(img, cvt)
        return img

    def spin_once(self):
        if not self.enabled:
            return
        frame = None
        with self._lock:
            if self._frame is not None:
                frame = self._frame.copy()
        if frame is not None:
            cv2.imshow(self._window_name, frame)
        cv2.waitKey(1)

    def close(self):
        if self.enabled:
            cv2.destroyAllWindows()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def publish_enable(pub, enabled: bool):
    msg = Boolean()
    msg.data = enabled
    pub.publish(msg)


def publish_twist(pub, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.linear.z = vz
    msg.angular.z = yaw_rate
    pub.publish(msg)


def wait_for_odom(state: FlightState, timeout_s: float) -> bool:
    start = time.time()
    while time.time() - start < timeout_s:
        if state.has_odom:
            return True
        time.sleep(0.05)
    return False


def loop_sleep(sec: float, viewer: CameraViewer):
    if viewer is not None:
        viewer.spin_once()
    time.sleep(sec)


def hold_position(pub_twist, state: FlightState, x_ref: float, y_ref: float, z_ref: float, sec: float, viewer: CameraViewer):
    period = 1.0 / CONTROL_HZ
    kp_xy = 0.9
    kp_z = 1.0
    end = time.time() + sec
    while time.time() < end:
        vx = clamp(kp_xy * (x_ref - state.x), -0.6, 0.6)
        vy = clamp(kp_xy * (y_ref - state.y), -0.6, 0.6)
        vz = clamp(kp_z * (z_ref - state.z), -0.6, 0.6)
        publish_twist(pub_twist, vx, vy, vz, 0.0)
        loop_sleep(period, viewer)


def move_to_point(
    pub_twist,
    state: FlightState,
    x_ref: float,
    y_ref: float,
    z_ref: float,
    viewer: CameraViewer,
    xy_tol: float = 0.08,
    z_tol: float = 0.10,
    stable_s: float = 1.0,
    kp_xy: float = 1.0,
    kp_z: float = 0.9,
    max_xy_speed: float = 0.9,
    max_z_speed: float = 0.6,
):
    period = 1.0 / CONTROL_HZ
    stable_since = None
    while True:
        ex = x_ref - state.x
        ey = y_ref - state.y
        z_err = z_ref - state.z
        dist_xy = math.hypot(ex, ey)

        vx = clamp(kp_xy * ex, -max_xy_speed, max_xy_speed)
        vy = clamp(kp_xy * ey, -max_xy_speed, max_xy_speed)
        vz = clamp(kp_z * z_err, -max_z_speed, max_z_speed)
        publish_twist(pub_twist, vx, vy, vz, 0.0)

        if dist_xy < xy_tol and abs(z_err) < z_tol:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since > stable_s:
                break
        else:
            stable_since = None
        loop_sleep(period, viewer)


def fly_circle(
    pub_twist,
    state: FlightState,
    center_x: float,
    center_y: float,
    radius_m: float,
    z_ref: float,
    viewer: CameraViewer,
):
    period = 1.0 / CONTROL_HZ
    w = CIRCLE_ANGULAR_SPEED_RAD_S
    kp_track = 1.2
    kp_z = 0.9
    max_xy_speed = 1.3
    max_z_speed = 0.6

    start_theta = math.atan2(state.y - center_y, state.x - center_x)
    start_t = time.time()
    duration = (2.0 * math.pi) / abs(w)
    end_t = start_t + duration

    while time.time() < end_t:
        t = time.time() - start_t
        theta = start_theta + w * t

        x_ref = center_x + radius_m * math.cos(theta)
        y_ref = center_y + radius_m * math.sin(theta)

        vx_ff = -radius_m * w * math.sin(theta)
        vy_ff = radius_m * w * math.cos(theta)

        ex = x_ref - state.x
        ey = y_ref - state.y
        z_err = z_ref - state.z

        vx = clamp(vx_ff + kp_track * ex, -max_xy_speed, max_xy_speed)
        vy = clamp(vy_ff + kp_track * ey, -max_xy_speed, max_xy_speed)
        vz = clamp(kp_z * z_err, -max_z_speed, max_z_speed)
        publish_twist(pub_twist, vx, vy, vz, 0.0)
        loop_sleep(period, viewer)

    # Brief settle at end of the circle before next phase.
    end_x = center_x + radius_m * math.cos(start_theta + 2.0 * math.pi)
    end_y = center_y + radius_m * math.sin(start_theta + 2.0 * math.pi)
    hold_position(pub_twist, state, end_x, end_y, z_ref, 0.5, viewer)


def main():
    node = Node()
    state = FlightState()

    odom_topic = f"/model/{MODEL_NAME}/odometry"
    belly_camera_topic = "/down_rgb_camera/image_raw"
    enable_topic = f"/model/{MODEL_NAME}/enable"
    cmd_topic = f"/model/{MODEL_NAME}/gazebo/command/twist"

    ok_sub = node.subscribe(Odometry, odom_topic, state.odom_cb)
    if not ok_sub:
        raise RuntimeError(f"Failed to subscribe to {odom_topic}")

    viewer = CameraViewer(node, belly_camera_topic)
    pub_enable = node.advertise(enable_topic, Boolean)
    pub_twist = node.advertise(cmd_topic, Twist)
    time.sleep(0.5)

    if not wait_for_odom(state, timeout_s=10.0):
        raise RuntimeError("No odometry received from Gazebo.")

    try:
        print(f"[INFO] Initial pose: x={state.x:.2f}, y={state.y:.2f}, z={state.z:.2f}")
        print("[INFO] Enabling multicopter velocity controller")
        publish_enable(pub_enable, True)
        loop_sleep(0.2, viewer)

        first_circle_start_x = BASE_X_M + FIRST_CIRCLE_RADIUS_M
        first_circle_start_y = BASE_Y_M
        second_circle_start_x = BASE_X_M + SECOND_CIRCLE_RADIUS_M
        second_circle_start_y = BASE_Y_M
        third_circle_start_x = BASE_X_M + THIRD_CIRCLE_RADIUS_M
        third_circle_start_y = BASE_Y_M

        # Phase 1: Takeoff to 2.5 m over base.
        print("[INFO] Takeoff phase: climbing to 2.5 m")
        move_to_point(
            pub_twist,
            state,
            BASE_X_M,
            BASE_Y_M,
            TAKEOFF_ALT_M,
            viewer,
            stable_s=1.5,
        )

        # Phase 2: Move to 1.0 m offset from base at 2.5 m.
        print("[INFO] Move phase: positioning at 1.0 m radius from base")
        move_to_point(
            pub_twist,
            state,
            first_circle_start_x,
            first_circle_start_y,
            TAKEOFF_ALT_M,
            viewer,
        )

        # Phase 3: First full circle around base at 2.5 m and radius 1.0 m.
        print("[INFO] Circle phase 1: radius 1.0 m at 2.5 m altitude")
        fly_circle(
            pub_twist,
            state,
            BASE_X_M,
            BASE_Y_M,
            FIRST_CIRCLE_RADIUS_M,
            TAKEOFF_ALT_M,
            viewer,
        )

        # Phase 4: Climb to 4.0 m.
        print("[INFO] Climb phase: moving to 4.0 m")
        move_to_point(
            pub_twist,
            state,
            first_circle_start_x,
            first_circle_start_y,
            HIGH_ALT_M,
            viewer,
        )

        # Phase 5: Move to 2.0 m offset from base at 4.0 m.
        print("[INFO] Move phase: positioning at 2.0 m radius from base")
        move_to_point(
            pub_twist,
            state,
            second_circle_start_x,
            second_circle_start_y,
            HIGH_ALT_M,
            viewer,
        )

        # Phase 6: Second full circle around base at 4.0 m and radius 2.0 m.
        print("[INFO] Circle phase 2: radius 2.0 m at 4.0 m altitude")
        fly_circle(
            pub_twist,
            state,
            BASE_X_M,
            BASE_Y_M,
            SECOND_CIRCLE_RADIUS_M,
            HIGH_ALT_M,
            viewer,
        )

        # Phase 7: Descend to 2.0 m while holding 2.0 m radius.
        print("[INFO] Descent phase: moving down to 2.0 m")
        move_to_point(
            pub_twist,
            state,
            third_circle_start_x,
            third_circle_start_y,
            MID_ALT_M,
            viewer,
        )

        # Phase 8: Third full circle around base at 2.0 m.
        print("[INFO] Circle phase 3: radius 2.0 m at 2.0 m altitude")
        fly_circle(
            pub_twist,
            state,
            BASE_X_M,
            BASE_Y_M,
            THIRD_CIRCLE_RADIUS_M,
            MID_ALT_M,
            viewer,
        )

        # Phase 9: Return over base center at 2.0 m.
        print("[INFO] Return-to-base phase: moving over base center")
        move_to_point(
            pub_twist,
            state,
            BASE_X_M,
            BASE_Y_M,
            MID_ALT_M,
            viewer,
        )

        # Phase 10: Hover over base for 2 seconds before descent.
        print("[INFO] Base-hover phase: holding for 2.0 seconds")
        hold_position(pub_twist, state, BASE_X_M, BASE_Y_M, MID_ALT_M, BASE_HOVER_BEFORE_LAND_S, viewer)

        # Phase 11: Land at the base center with a slower vertical profile.
        print("[INFO] Landing phase: slow descent on base")
        period = 1.0 / CONTROL_HZ
        while state.z > LAND_SETTLE_Z_M:
            vx = clamp(1.0 * (BASE_X_M - state.x), -0.5, 0.5)
            vy = clamp(1.0 * (BASE_Y_M - state.y), -0.5, 0.5)
            vz = clamp(0.8 * (LAND_SETTLE_Z_M - state.z), -0.35, 0.08)
            publish_twist(pub_twist, vx, vy, vz, 0.0)
            loop_sleep(period, viewer)

        hold_position(pub_twist, state, BASE_X_M, BASE_Y_M, LAND_SETTLE_Z_M, 1.5, viewer)
        publish_twist(pub_twist, 0.0, 0.0, 0.0, 0.0)
        loop_sleep(0.3, viewer)
        publish_enable(pub_enable, False)
        print(f"[DONE] Mission complete. Final pose: x={state.x:.2f}, y={state.y:.2f}, z={state.z:.2f}")
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
