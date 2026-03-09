#!/usr/bin/env python3
"""
save_3d_map.py — Save the 3D OctoMap and Point Cloud from a running mapping session.

Subscribes to:
  /octomap_binary              → saves as .bt  (native OctoMap binary tree)
  /octomap_point_cloud_centers → saves as .pcd  (ASCII PCD for ML pipelines)

Usage (while mapping mission is active):
    python3 scripts/save_3d_map.py

Output files land in  maps/  with a timestamp, e.g.:
    maps/arena_3d_map_20260223_023500.bt
    maps/arena_3d_cloud_20260223_023500.pcd
"""

import os
import sys
import struct
import datetime
import threading

# Force unbuffered output so logs appear immediately
os.environ.setdefault("PYTHONUNBUFFERED", "1")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from octomap_msgs.msg import Octomap
from sensor_msgs.msg import PointCloud2

MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps")
TIMEOUT_SEC = 45.0  # max seconds to wait for each topic


class MapSaver(Node):
    def __init__(self, stamp: str):
        super().__init__("map_saver_3d")
        self.stamp = stamp
        self.bt_saved = False
        self.pcd_saved = False
        self._lock = threading.Lock()

        # Subscribe with MULTIPLE QoS profiles to handle both latched and volatile publishers.
        # The octomap_server publishes with transient-local (latched), but under heavy
        # simulation load the DDS matching can fail. We subscribe with both profiles.

        # Profile 1: Transient-local (for latched publishers)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Profile 2: Volatile / best-effort (fallback)
        volatile_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.get_logger().info("Creating subscriptions (latched + volatile QoS) …")

        self.sub_octomap_l = self.create_subscription(
            Octomap, "/octomap_binary", self._on_octomap, latched_qos
        )
        self.sub_octomap_v = self.create_subscription(
            Octomap, "/octomap_binary", self._on_octomap, volatile_qos
        )
        self.sub_pc_l = self.create_subscription(
            PointCloud2, "/octomap_point_cloud_centers", self._on_pointcloud, latched_qos
        )
        self.sub_pc_v = self.create_subscription(
            PointCloud2, "/octomap_point_cloud_centers", self._on_pointcloud, volatile_qos
        )

        self.get_logger().info("Waiting for OctoMap + PointCloud messages …")
        self._timer = self.create_timer(2.0, self._tick)
        self._elapsed = 0.0

    # ── OctoMap binary (.bt) ──────────────────────────────────────────
    def _on_octomap(self, msg: Octomap):
        with self._lock:
            if self.bt_saved:
                return
            self.bt_saved = True

        os.makedirs(MAPS_DIR, exist_ok=True)
        path = os.path.join(MAPS_DIR, f"arena_3d_map_{self.stamp}.bt")

        # Write the OcTree header expected by the octomap library, then binary data.
        with open(path, "wb") as f:
            tree_id = msg.id if msg.id else "OcTree"
            header = (
                f"# Octomap OcTree binary file\n"
                f"id {tree_id}\n"
                f"size {len(msg.data)}\n"
                f"res {msg.resolution}\n"
                f"data\n"
            )
            f.write(header.encode("ascii"))
            f.write(bytes(msg.data))

        size_kb = os.path.getsize(path) / 1024.0
        self.get_logger().info(f"✓ OctoMap saved: {path}  ({size_kb:.1f} KB)")
        self._maybe_quit()

    # ── Point Cloud (.pcd) ────────────────────────────────────────────
    def _on_pointcloud(self, msg: PointCloud2):
        with self._lock:
            if self.pcd_saved:
                return
            self.pcd_saved = True

        os.makedirs(MAPS_DIR, exist_ok=True)
        path = os.path.join(MAPS_DIR, f"arena_3d_cloud_{self.stamp}.pcd")

        points = self._parse_xyz(msg)
        self._write_pcd(path, points)

        self.get_logger().info(f"✓ PointCloud saved: {path}  ({len(points)} points)")
        self._maybe_quit()

    @staticmethod
    def _parse_xyz(msg: PointCloud2):
        """Extract (x, y, z) tuples from a PointCloud2 message."""
        offsets = {}
        for field in msg.fields:
            if field.name in ("x", "y", "z"):
                offsets[field.name] = field.offset

        if len(offsets) != 3:
            return []

        points = []
        data = bytes(msg.data)
        for i in range(msg.width * msg.height):
            base = i * msg.point_step
            x = struct.unpack_from("f", data, base + offsets["x"])[0]
            y = struct.unpack_from("f", data, base + offsets["y"])[0]
            z = struct.unpack_from("f", data, base + offsets["z"])[0]
            # Skip NaN points
            if x != x or y != y or z != z:
                continue
            points.append((x, y, z))
        return points

    @staticmethod
    def _write_pcd(path: str, points: list):
        """Write an ASCII PCD file (compatible with Open3D, PCL, etc.)."""
        with open(path, "w") as f:
            f.write("# .PCD v0.7 - 3D Point Cloud Data\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            for x, y, z in points:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

    # ── Timeout / completion checks ──────────────────────────────────
    def _tick(self):
        self._elapsed += 2.0
        with self._lock:
            bt = self.bt_saved
            pcd = self.pcd_saved
        status = []
        if not bt:
            status.append("OctoMap")
        if not pcd:
            status.append("PointCloud")
        if status:
            self.get_logger().info(
                f"  waiting for: {', '.join(status)}  ({self._elapsed:.0f}s / {TIMEOUT_SEC:.0f}s)"
            )
        if self._elapsed >= TIMEOUT_SEC:
            missing = ", ".join(status) if status else "none"
            self.get_logger().warn(
                f"Timed out after {TIMEOUT_SEC}s. Missing: {missing}"
            )
            if bt or pcd:
                self.get_logger().info("Partial export — see above for saved files.")
            else:
                self.get_logger().error(
                    "No data received. Is octomap_server running?  "
                    "Make sure the mapping mission is active."
                )
            raise SystemExit(0)

    def _maybe_quit(self):
        with self._lock:
            if self.bt_saved and self.pcd_saved:
                self.get_logger().info("══════════════════════════════════════════════")
                self.get_logger().info("  3D Map Export Complete!")
                self.get_logger().info(f"  Files in: {MAPS_DIR}")
                self.get_logger().info("")
                self.get_logger().info("  Load in Python for training:")
                self.get_logger().info("    import open3d as o3d")
                self.get_logger().info("    pcd = o3d.io.read_point_cloud('<file>.pcd')")
                self.get_logger().info("    points = np.asarray(pcd.points)  # Nx3")
                self.get_logger().info("══════════════════════════════════════════════")
                raise SystemExit(0)


def main():
    rclpy.init()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    node = MapSaver(stamp)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
