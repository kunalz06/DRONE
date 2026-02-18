# Integrated Proceeding Plan

This plan combines `gazebosimulation.md` and `missiondev.md` into one execution sequence from simulation-first development to hardware deployment.

## 1. Target Outcome
- Build a ROS 2 Jazzy drone stack that can complete the arena mission end-to-end:
  - Start only at battery >= 80%
  - Perform takeoff, mapping/search, boundary-safe navigation, RTL, docking, and sync
  - Return with mission completion or failsafe at battery <= 30%

## 2. Phase Order I Will Follow

### Phase A: Environment and Workspace Baseline
- Install Ubuntu 24.04 on laptop and Raspberry Pi 5.
- Install ROS 2 Jazzy on both systems.
- Install Gazebo Harmonic on the laptop.
- Create workspace and packages:
  - Workspace: `~/drone_ws`
  - Core package: `arena_drone_system` (ament_python)
  - Simulation package (if separate): `arena_drone_sim`
- Install dependencies:
  - `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `cv_bridge`, `mavros_msgs`, `ros_gz_bridge`, OpenCV, NumPy

### Phase B: Gazebo Arena and Assets
- Add `worlds/arena.sdf` using the Mars terrain world layout.
- Add `scripts/generate_mars_assets.py` to generate:
  - `heightmap.png`
  - `mars_texture.png`
- Run asset generation:
  - `cd ~/drone_ws/src/arena_drone_sim/scripts`
  - `python3 generate_mars_assets.py`
- Keep/verify drone SDF compatibility with ROS 2 + Gazebo plugins.
- Ensure required simulation systems/plugins are active:
  - Physics, sensors, scene broadcaster, user commands

### Phase C: Bridge and Launch Integration
- Configure `ros_gz_bridge parameter_bridge` for at least:
  - `/scan`
  - `/camera`
  - `/odom`
- Add/verify launch files:
  - simulation launch (`sim.launch.py`)
  - mission launch (`drone_mission.launch.py`)
- Validate simulation startup:
  - `cd ~/drone_ws`
  - `colcon build --symlink-install`
  - `source install/setup.bash`
  - `ros2 launch arena_drone_sim sim.launch.py`

### Phase D: FSM Mission Implementation (Core Autonomy)
- Implement mission FSM node with these states and transitions:
  - `WAITING_FOR_CHARGE`: wait until `/mavros/battery >= 80%`
  - `TAKEOFF`: P-control climb to `z = 3.0 m`
  - `MAPPING_SEARCH`:
    - obstacle avoidance from LiDAR within 1.0 m
    - yellow boundary detection via HSV + reversal maneuver
    - search path execution and image capture
  - `RTL`: trigger on battery <= 30% or timer expiration, return to `(0,0)`
  - `DOCKING`: ArUco-based XY alignment and descend when pixel error < 20 px
  - `SYNCING`: detect landing (`z < 0.15 m`), start transfer thread
- Add per-state watchdogs/timeouts and explicit transition logs.

### Phase E: Wireless and Data Sync
- Configure Pi network using `nmcli`:
  - static IP
  - preferred base station SSID
- Configure passwordless SSH (Pi -> laptop) with RSA keys and `ssh-copy-id`.
- Implement autonomous transfer using `rsync --remove-source-files` after landing.

### Phase F: Hardware Integration and Field Validation
- Mount Pi 5, LiDAR, and camera on the airframe.
- Connect Pi to Pixhawk (USB/UART) and run MAVROS.
- Calibrate IMU and magnetometer.
- Execute mission launch and validate full cycle telemetry.

## 3. Verification Gates (Must Pass Before Next Phase)
- Gate 1: Simulation world loads with terrain, boundaries, and base station.
- Gate 2: Sensor topics bridge correctly into ROS 2.
- Gate 3: FSM transitions work in simulation without manual intervention.
- Gate 4: RTL and docking succeed repeatedly.
- Gate 5: Sync process completes and source files are cleared on drone side.
- Gate 6: Hardware run matches simulation behavior and safety constraints.

## 4. Immediate Execution Sequence (First Build)
1. Build workspace/package skeleton and dependency install.
2. Add arena world + terrain generator, generate assets.
3. Bring up Gazebo + bridge and confirm topics.
4. Implement FSM state-by-state, testing each state before chaining.
5. Add networking + sync automation.
6. Move to hardware with incremental validation (hover, nav, docking, full mission).

## 5. Assumptions to Confirm Early
- Final workspace path (`~/drone_ws` vs existing workspace naming).
- Actual drone model/plugin stack (native Gazebo control vs PX4/ArduPilot SITL).
- Camera orientation for ArUco docking (downward camera availability).
- Mission timer duration and expected image/data payload size.
