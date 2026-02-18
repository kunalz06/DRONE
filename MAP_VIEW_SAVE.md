# SLAM Map: View and Save Guide

This guide shows how to run mapping, view the live map, and save the final map files.

## 1) Start Gazebo World

Open terminal 1:

```bash
cd /home/kunal/Desktop/drone
HOME=/tmp gz sim -r worlds/arena.sdf
```

## 2) Start SLAM + Bridge + Mission

Open terminal 2:

```bash
cd /home/kunal/Desktop/drone
source /opt/ros/jazzy/setup.bash
HOME=/tmp ros2 launch /home/kunal/Desktop/drone/launch/slam_arena_mission.launch.py
```

Notes:
- RViz opens by default and shows the live map.
- Failsafe logic remains in code, but automatic failsafe trigger is disabled for mapping runs.

## 3) View the Live Map

- In RViz, confirm `Map` display is subscribed to `/map`.
- Optional topic check:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /map --once
```

## 4) Save the Built Map

After mapping is complete (or whenever you want to snapshot the map):

```bash
mkdir -p /home/kunal/Desktop/drone/maps
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli -f /home/kunal/Desktop/drone/maps/arena_map --ros-args -p map_subscribe_transient_local:=true
```

Saved outputs:
- `/home/kunal/Desktop/drone/maps/arena_map.yaml`
- `/home/kunal/Desktop/drone/maps/arena_map.pgm`

## 5) Optional: Save With Timestamp

```bash
mkdir -p /home/kunal/Desktop/drone/maps
source /opt/ros/jazzy/setup.bash
STAMP=$(date +%Y%m%d_%H%M%S)
ros2 run nav2_map_server map_saver_cli -f /home/kunal/Desktop/drone/maps/arena_map_${STAMP} --ros-args -p map_subscribe_transient_local:=true
```
