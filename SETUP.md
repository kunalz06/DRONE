# DRONE Project Setup Guide

This guide gets the project running from a clean Ubuntu machine.

## 1) Prerequisites

- Ubuntu 24.04 (recommended)
- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- Gazebo Harmonic with `gz sim`

## 2) Get the Repository

```bash
cd ~/Desktop
git clone https://github.com/kunalz06/DRONE.git drone
cd drone
```

If you already have this folder at `/home/kunal/Desktop/drone`, just `cd` there.

## 3) Install Required Packages

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-map-server \
  ros-jazzy-cv-bridge \
  ros-jazzy-rviz2 \
  ros-jazzy-actuator-msgs \
  python3-opencv \
  python3-numpy \
  python3-gz-transport13 \
  python3-gz-msgs10
```

## 4) Set Environment (each new terminal)

```bash
source /opt/ros/jazzy/setup.bash
```

Optional convenience:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 5) Quick Validation

```bash
gz sim --versions
ros2 pkg list | rg "ros_gz_bridge|slam_toolbox|nav2_map_server"
python3 -c "import cv2, numpy"
```

## 6) Run Base Gazebo + Autonomous Mission

Terminal 1:

```bash
cd /home/kunal/Desktop/drone
HOME=/tmp gz sim -r worlds/px4_x500_mission_test.sdf
```

Terminal 2:

```bash
cd /home/kunal/Desktop/drone
HOME=/tmp python3 scripts/fly_takeoff_hover_land.py
```

## 7) Run Arena SLAM Mission

Terminal 1:

```bash
cd /home/kunal/Desktop/drone
HOME=/tmp gz sim -r worlds/arena.sdf
```

Terminal 2:

```bash
cd /home/kunal/Desktop/drone
source /opt/ros/jazzy/setup.bash
HOME=/tmp ros2 launch /home/kunal/Desktop/drone/launch/slam_arena_mission.launch.py
```

This starts:
- `ros_gz_bridge` using `config/arena_bridge_topics.yaml`
- `slam_toolbox` using `config/slam_toolbox_arena.yaml`
- mission node `scripts/slam_boundary_mapping_mission.py`
- RViz map view

## 8) Optional Manual Override

Terminal 3:

```bash
cd /home/kunal/Desktop/drone
source /opt/ros/jazzy/setup.bash
python3 scripts/manual_control_link.py
```

Key controls:
- `m`: manual mode on
- `n`: return to autonomous
- `w/s/a/d`: XY motion
- `i/k`: up/down
- `j/l`: yaw
- `space`: zero velocities

## 9) Save Map Output

```bash
cd /home/kunal/Desktop/drone
mkdir -p maps
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli -f maps/arena_map --ros-args -p map_subscribe_transient_local:=true
```

Outputs:
- `maps/arena_map.yaml`
- `maps/arena_map.pgm`

## 10) Common Issues

- `could not find package ...`: ROS package missing; re-run apt install list above.
- No RViz/camera window: ensure desktop session has `DISPLAY`/`WAYLAND_DISPLAY`.
- No `/map`: make sure Gazebo is running first, then launch `slam_arena_mission.launch.py`.
- Permission issues on scripts:

```bash
chmod +x scripts/*.py launch/*.py
```
