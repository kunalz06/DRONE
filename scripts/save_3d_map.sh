#!/bin/bash
# save_3d_map.sh — Wrapper to run the Python 3D map exporter.
# Saves OctoMap (.bt) and PointCloud (.pcd) from an active mapping session.
#
# Usage:
#   ./scripts/save_3d_map.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash

echo "Starting 3D map export …"
python3 "${SCRIPT_DIR}/save_3d_map.py"
