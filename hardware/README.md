# Hardware Deployment Guide

This guide covers deploying the autonomous drone system on real hardware with Raspberry Pi 5 and Pixhawk.

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Software Setup](#software-setup)
3. [System Architecture](#system-architecture)
4. [Configuration](#configuration)
5. [Pre-flight Checklist](#pre-flight-checklist)
6. [Running Missions](#running-missions)
7. [Troubleshooting](#troubleshooting)
8. [Safety Guidelines](#safety-guidelines)

---

## Hardware Requirements

### Core Components

| Component | Specification | Purpose |
|-----------|--------------|---------|
| **Raspberry Pi 5** | 8GB RAM recommended | Main computer |
| **Pixhawk 6C/6X** | PX4-compatible flight controller | Flight control |
| **LiDAR** | RPLidar A1/A2 (360°) | SLAM mapping |
| **Camera** | USB webcam (720p+) | Boundary detection |
| **Battery** | 4S LiPo 5000mAh+ | Power supply |
| **GPS** | u-blox NEO-M8N | Positioning |
| **Telemetry** | 433MHz/915MHz radio | Ground station link |

### Wiring Diagram

```
┌─────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5                       │
├─────────────────────────────────────────────────────────┤
│  USB Port 1  ─────► Pixhawk (TELEM2/USB)               │
│  USB Port 2  ─────► RPLidar                             │
│  USB Port 3  ─────► USB Camera (belly mount)           │
│  GPIO        ─────► Status LEDs / Buzzer               │
│  CSI Port    ─────► (Optional) Raspberry Pi Camera     │
└─────────────────────────────────────────────────────────┘
                           │
                           │ MAVLink/MAVROS
                           ▼
┌─────────────────────────────────────────────────────────┐
│                    Pixhawk 6C                           │
├─────────────────────────────────────────────────────────┤
│  TELEM1   ─────► GPS Module                             │
│  TELEM2   ─────► Raspberry Pi (MAVLink)                │
│  RC IN    ─────► RC Receiver (for manual override)     │
│  ESC 1-4  ─────► Motor ESCs                             │
│  SERVO    ─────► Gimbal (optional)                     │
│  BATTERY  ─────► Power Module                           │
└─────────────────────────────────────────────────────────┘
```

---

## Software Setup

### Quick Install (Recommended)

```bash
# 1. Clone the repository
git clone https://github.com/kunalz06/DRONE.git ~/drone
cd ~/drone

# 2. Run the automated setup script
chmod +x hardware/scripts/setup_raspberry_pi.sh
./hardware/scripts/setup_raspberry_pi.sh --full
```

### Manual Installation

If you prefer manual installation or need to customize:

```bash
# 1. Install ROS 2 Jazzy
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-ros-base

# 2. Install MAVROS
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras

# 3. Install dependencies
sudo apt install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server \
    ros-jazzy-cv-bridge \
    python3-opencv \
    python3-numpy

# 4. Setup workspace
mkdir -p ~/drone_ws/src
ln -s ~/drone ~/drone_ws/src/drone
cd ~/drone_ws
colcon build --symlink-install
```

### PX4 Configuration

1. **Install QGroundControl** on your computer
2. **Flash PX4 firmware** to Pixhawk
3. **Configure parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `SYS_HITL` | 0 | Disable HITL |
| `MAV_1_MODE` | Onboard | Enable onboard computer link |
| `MAV_1_RATE` | 50 | Telemetry rate (Hz) |
| `SER_TEL2_BAUD` | 57600 | Telemetry baud rate |
| `CBRK_USB_CHK` | 894415 | Disable USB check for arm |
| `COM_RCL_EXCEPT` | 4 | Allow arm without RC |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Mission Stack                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │   Mission   │───►│   Safety    │───►│   MAVROS    │        │
│  │    Node     │    │   Manager   │    │   Interface │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│         │                  │                  │                │
│         ▼                  ▼                  ▼                │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │    SLAM     │    │   Sensor    │    │   Flight    │        │
│  │   Toolbox   │    │   Fusion    │    │   Control   │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼ MAVLink
                    ┌─────────────────┐
                    │    Pixhawk      │
                    │  (PX4 Firmware) │
                    └─────────────────┘
```

### Node Responsibilities

| Node | Purpose |
|------|---------|
| **Mission Node** | State machine, waypoint navigation, boundary detection |
| **Safety Manager** | Geofence, battery monitoring, failsafe triggers |
| **MAVROS Interface** | Communication bridge to PX4, setpoint publishing |
| **SLAM Toolbox** | Real-time 2D mapping from LiDAR |
| **Sensor Fusion** | IMU/GPS integration, state estimation |

---

## Configuration

### Mission Parameters

Edit `config/mission_params.yaml`:

```yaml
mission:
  target_altitude: 4.0        # Mapping altitude (m)
  control_rate: 20.0          # Control loop frequency (Hz)

velocity_limits:
  max_xy_speed: 1.45          # Max horizontal speed (m/s)
  max_z_speed: 0.6            # Max vertical speed (m/s)

battery:
  low_warn_percent: 25.0      # Battery warning threshold
  critical_percent: 12.0      # Emergency landing threshold

safety:
  max_distance_from_base: 150.0  # Geofence radius (m)
  max_altitude: 50.0             # Max altitude (m)
```

### Hardware-Specific Config

Edit `hardware/config/hardware_safety.yaml`:

```yaml
safety_manager:
  max_distance_from_home: 150.0
  battery_critical_threshold: 15.0
  comm_loss_timeout: 3.0

geofence:
  fence_type: "circular"
  radius: 150.0
  breach_action: "rtl"
```

### Sensor Calibration

1. **IMU Calibration:**
   ```bash
   # On Pixhawk, use QGroundControl
   # Sensors → Calibrate → Gyroscope/Accelerometer
   ```

2. **Magnetometer Calibration:**
   ```bash
   # Perform compass calibration outdoors
   # Sensors → Calibrate → Compass
   ```

3. **Camera Calibration:**
   ```bash
   ros2 run camera_calibration cameracalibrator \
     --size 8x6 --square 0.108 \
     /camera/image_raw
   ```

---

## Pre-flight Checklist

### Before Each Flight

```
□ Battery charged (> 80%)
□ Propellers installed correctly
□ GPS has 3D fix (LED solid)
□ Compass calibrated (if new location)
□ SD card inserted (for data logging)
□ RC controller paired (for manual override)
□ Clear takeoff area
□ Weather conditions acceptable (wind < 10 m/s)
```

### System Check Commands

```bash
# Check all sensors
ros2 topic hz /mavros/imu/data
ros2 topic hz /scan
ros2 topic hz /mavros/global_position/global

# Check battery
ros2 topic echo /mavros/battery --once

# Check home position set
ros2 topic echo /mavros/home_position/home --once

# Monitor system
~/monitor_drone.sh
```

---

## Running Missions

### Start Mission

```bash
# Quick start
~/start_mission.sh

# With custom parameters
ros2 launch ~/drone/hardware/launch/hardware_mission.launch.py \
    use_rviz:=true \
    use_slam:=true
```

### Manual Override

During autonomous operation, you can take manual control:

```bash
# In separate terminal
python3 ~/drone/scripts/manual_control_link.py

# Keys:
#   m - Enable manual mode
#   n - Return to autonomous
#   w/s/a/d - Move
#   i/k - Up/Down
#   space - Stop
#   b - Return to base
```

### Save Map

After mapping mission:

```bash
mkdir -p ~/drone/maps
ros2 run nav2_map_server map_saver_cli -f ~/drone/maps/mission_map \
    --ros-args -p map_subscribe_transient_local:=true
```

---

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| **MAVROS not connecting** | Check USB connection: `ls /dev/pixhawk` |
| **No GPS fix** | Wait outdoors for 2-5 minutes |
| **Cannot arm** | Check pre-arm errors in QGC |
| **LiDAR not publishing** | Check: `ls /dev/rplidar` and permissions |
| **High CPU usage** | Reduce control_rate or SLAM resolution |
| **Position drift** | Recalibrate IMU/magnetometer |

### Log Files

```bash
# ROS logs
~/.ros/log/

# MAVROS logs
ros2 topic echo /mavros/state

# System logs
journalctl -u drone-mission.service -f
```

### Debug Mode

```bash
# Run with verbose output
ros2 launch hardware_mission.launch.py log_level:=debug

# Check specific topics
ros2 topic hz /scan
ros2 topic hz /mavros/local_position/odom
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

---

## Safety Guidelines

### Critical Safety Rules

1. **Never arm indoors** without propellers removed
2. **Always have RC controller** ready for manual override
3. **Check failsafe settings** before each flight
4. **Maintain line of sight** during operations
5. **Monitor battery** continuously during flight
6. **Test new configurations** in simulation first

### Failsafe Actions

| Condition | Default Action |
|-----------|---------------|
| Battery < 15% | Automatic RTL |
| GPS loss > 5s | Hover and wait |
| Comm loss > 3s | RTL |
| Geofence breach | RTL |
| RC loss | RTL after 2s |

### Emergency Procedures

1. **Loss of Control:**
   - Switch RC to manual mode immediately
   - Fly to safe altitude
   - Land manually

2. **Battery Emergency:**
   - Do NOT attempt RTL if battery critically low
   - Land immediately at current position

3. **Flyaway:**
   - Switch to LAND mode via RC
   - Cut throttle if necessary (last resort)

---

## Advanced Configuration

### Multi-Vehicle Setup

For multiple drones, modify launch arguments:

```bash
# Vehicle 1
ros2 launch hardware_mission.launch.py \
    vehicle_id:=1 \
    mavros_namespace:=/drone1

# Vehicle 2  
ros2 launch hardware_mission.launch.py \
    vehicle_id:=2 \
    mavros_namespace:=/drone2
```

### ROS_DOMAIN_ID

For multi-vehicle systems without interference:

```bash
# On each vehicle, set unique domain ID
export ROS_DOMAIN_ID=1  # Vehicle 1
export ROS_DOMAIN_ID=2  # Vehicle 2
```

### DDS Configuration

For better network performance:

```bash
# Install Cyclone DDS
sudo apt install ros-jazzy-rmw-cyclonedds-cpp

# Set as default
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## Support

For issues and questions:
- GitHub Issues: https://github.com/kunalz06/DRONE/issues
- Documentation: See `MCP.md` and `execution_plan.md`
