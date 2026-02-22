#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

INTEGRATION=0
if [[ "${1:-}" == "--integration" ]]; then
  INTEGRATION=1
fi

info() {
  echo "[check] $*"
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[error] Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd python3
require_cmd gz

info "Python syntax checks"
python3 - <<'PY'
import ast
import pathlib

targets = list(pathlib.Path("scripts").glob("*.py")) + list(pathlib.Path("launch").glob("*.py"))
for path in sorted(targets):
    source = path.read_text(encoding="utf-8")
    ast.parse(source, filename=str(path))
    print(f"[syntax] {path}: OK")
PY

info "YAML validation"
python3 - <<'PY'
import pathlib
import yaml

files = [
    "config/arena_bridge_topics.yaml",
    "config/slam_toolbox_arena.yaml",
]
map_file = pathlib.Path("maps/arena_map.yaml")
if map_file.exists():
    files.append(str(map_file))

for path in files:
    with open(path, "r", encoding="utf-8") as f:
        yaml.safe_load(f)
    print(f"[yaml] {path}: OK")
PY

info "SDF validation"
gz sdf -k worlds/arena.sdf
gz sdf -k models/px4_x500_lidar_rgb/model.sdf

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u

  require_cmd ros2
  info "ROS launch parse checks"
  PYTHONDONTWRITEBYTECODE=1 HOME=/tmp ros2 launch "$ROOT_DIR/launch/slam_arena_mission.launch.py" --show-args >/dev/null
else
  echo "[warn] /opt/ros/jazzy/setup.bash not found; skipping ROS checks."
fi

if [[ "$INTEGRATION" -eq 1 ]]; then
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "[error] ROS environment is required for --integration." >&2
    exit 1
  fi

  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
  require_cmd ros2

  info "Integration smoke (Gazebo server + ROS launch)"
  LOG_DIR="/tmp/drone_fullcheck_logs_$$"
  mkdir -p "$LOG_DIR"

  HOME=/tmp gz sim -r -s "$ROOT_DIR/worlds/arena.sdf" >"$LOG_DIR/gz.log" 2>&1 &
  GZ_PID=$!
  sleep 6

  PYTHONDONTWRITEBYTECODE=1 HOME=/tmp ros2 launch "$ROOT_DIR/launch/slam_arena_mission.launch.py" use_rviz:=false >"$LOG_DIR/launch.log" 2>&1 &
  LAUNCH_PID=$!
  sleep 8

  PYTHONDONTWRITEBYTECODE=1 HOME=/tmp timeout 12s ros2 topic echo /map --once >"$LOG_DIR/map_once.log" 2>&1 || true

  kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
  sleep 1
  kill -TERM "$LAUNCH_PID" >/dev/null 2>&1 || true

  kill -INT "$GZ_PID" >/dev/null 2>&1 || true
  sleep 1
  kill -TERM "$GZ_PID" >/dev/null 2>&1 || true

  echo "[check] Integration logs: $LOG_DIR"
  if grep -q "header:" "$LOG_DIR/map_once.log"; then
    echo "[check] /map probe: received at least one OccupancyGrid message."
  else
    echo "[warn] /map probe: no message captured within timeout."
  fi
  echo "[check] Launch log tail:"
  tail -n 40 "$LOG_DIR/launch.log" || true
fi

info "All checks completed"
