# PX4 X500 Sim Run Commands

This file shows the exact commands to run the drone simulation and explains what the mission code does.

## 1) Start Gazebo (GUI)

Open terminal 1:

```bash
cd /home/kunal/Desktop/drone
HOME=/tmp gz sim -r worlds/px4_x500_mission_test.sdf
```

What this does:
- Loads `worlds/px4_x500_mission_test.sdf`
- Spawns the drone model `models/px4_x500_lidar_rgb/model.sdf`
- Starts Gazebo in GUI mode (remove `-r` if you want paused startup)

Arena run (same drone, full arena world):

```bash
cd /home/kunal/Desktop/drone
HOME=/tmp gz sim -r worlds/arena.sdf
```

## 2) Run Autonomous Flight Mission

Open terminal 2:

```bash
cd /home/kunal/Desktop/drone
HOME=/tmp python3 scripts/fly_takeoff_hover_land.py
```

What this script does:
- Subscribes to odometry: `/model/px4_x500_lidar_rgb/odometry`
- Subscribes to belly camera feed: `/down_rgb_camera/image_raw`
- Shows live belly camera window during flight (if display server is available)
- Enables velocity control: `/model/px4_x500_lidar_rgb/enable`
- Sends velocity setpoints: `/model/px4_x500_lidar_rgb/gazebo/command/twist`

## 3) Mission Behavior (Code Logic)

Inside `scripts/fly_takeoff_hover_land.py`, the mission is:

1. Takeoff:
- Climbs to `2.5 m` over the home base `(0, 0)`

2. Move to first orbit radius:
- Moves to `(1.0, 0.0)` at `2.5 m`

3. Circle 1:
- Completes one full circle around base `(0, 0)` with radius `1.0 m` at `2.5 m`

4. Climb:
- Climbs to `4.0 m`

5. Move to second orbit radius:
- Moves to `(2.0, 0.0)` at `4.0 m`

6. Circle 2:
- Completes one full circle around base `(0, 0)` with radius `2.0 m` at `4.0 m`

7. Descend:
- Descends to `2.0 m` while staying at `2.0 m` orbit radius

8. Circle 3:
- Completes another full circle around base `(0, 0)` with radius `2.0 m` at `2.0 m`

9. Return + Hover:
- Returns over base `(0, 0)` at `2.0 m`
- Hovers for `2.0 s`

10. Slow Land:
- Descends slowly to the base pad and disables the controller

## 4) Expected Console Output

You should see lines like:

```text
[INFO] Takeoff phase: climbing to 2.5 m
[INFO] Move phase: positioning at 1.0 m radius from base
[INFO] Circle phase 1: radius 1.0 m at 2.5 m altitude
[INFO] Climb phase: moving to 4.0 m
[INFO] Move phase: positioning at 2.0 m radius from base
[INFO] Circle phase 2: radius 2.0 m at 4.0 m altitude
[INFO] Circle phase 3: radius 2.0 m at 2.0 m altitude
[INFO] Landing phase: slow descent on base
[DONE] Mission complete. Final pose: x=..., y=..., z=...
```

## 5) Stop Simulation

In terminal 1 (Gazebo), press:

```text
Ctrl+C
```
