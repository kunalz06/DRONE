# Hardware Deployment Guide

## Complete Guide for Deploying the Drone System on Real Hardware

**Version:** 2.0.0  
**Last Updated:** February 2025  
**Target Platform:** Raspberry Pi 5 + Pixhawk 6C/6X + Ubuntu 24.04

---

## Table of Contents

1. [Hardware Requirements](#1-hardware-requirements)
2. [Software Prerequisites](#2-software-prerequisites)
3. [Quick Start](#3-quick-start)
4. [Detailed Setup](#4-detailed-setup)
5. [Configuration](#5-configuration)
6. [Testing](#6-testing)
7. [Operation](#7-operation)
8. [Troubleshooting](#8-troubleshooting)
9. [Safety Guidelines](#9-safety-guidelines)

---

## 1. Hardware Requirements

### 1.1 Required Components

| Component | Specification | Purpose |
|-----------|--------------|---------|
| **Flight Controller** | Pixhawk 6C or 6X | Autopilot with PX4 firmware |
| **Companion Computer** | Raspberry Pi 5 (8GB) | Mission control and sensor processing |
| **LiDAR** | RPLidar A2/A3 or equivalent | 2D mapping and obstacle detection |
| **Camera (Front)** | Raspberry Pi Camera V3 or USB webcam | Forward vision / boundary detection |
| **Camera (Down)** | Raspberry Pi Camera V3 or USB webcam | Belly camera for landing / boundary |
| **GPS** | u-blox NEO-M8N or better | Position estimation |
| **Telemetry Radio** | Holybro SiK Radio 433MHz/915MHz | Ground station communication |
| **Power Module** | Holybro PM02 or PM07 | Power distribution and monitoring |
| **Battery** | 4S LiPo 5000mAh+ | Main power source |
| **Motors** | 4x 920KV brushless | Propulsion |
| **ESCs** | 4x 30A (DShot capable) | Motor control |
| **Propellers** | 10-12 inch | Thrust generation |
| **Frame** | F450 or similar quadcopter | Airframe |

### 1.2 Recommended Additions

- **Time-of-Flight Sensor**: Garmin Lidar-Lite or Benewake TF-Luna for precise altitude
- **Buzzer**: Pixhawk compatible buzzer for status alerts
- **LED Strip**: WS2812B for visual status indication
- **Safety Switch**: Required for arming

### 1.3 Wiring Diagram

```
                            ┌─────────────────────────────────────┐
                            │           Raspberry Pi 5            │
                            │                                     │
    ┌─────────┐             │  USB ──────── Pixhawk (MAVLink)    │
    │  LiDAR  │─────────────│  GPIO ─────── Status LED           │
    └─────────┘   USB       │  CSI/USB ──── Front Camera         │
                            │  CSI/USB ──── Down Camera          │
    ┌─────────┐             │  UART ─────── Optional ToF         │
    │  GPS    │─────────────│                                     │
    └─────────┘   UART      └─────────────────────────────────────┘
                                        │
                                        │ USB/UART
                                        ▼
                            ┌─────────────────────────────────────┐
                            │           Pixhawk 6C/6X             │
                            │                                     │
                            │  MAIN ────── ESC 1 (Front Right)    │
                            │  AUX 1 ───── ESC 2 (Rear Left)      │
                            │  AUX 2 ───── ESC 3 (Front Left)     │
                            │  AUX 3 ───── ESC 4 (Rear Right)     │
                            │                                     │
                            │  GPS1 ─────── GPS Module            │
                            │  TELEM1 ───── Telemetry Radio       │
                            │  POWER1 ───── Power Module          │
                            │  BUZZER ───── Status Buzzer         │
                            │  SAFETY ───── Safety Switch         │
                            └─────────────────────────────────────┘
```

---

## 2. Software Prerequisites

### 2.1 Operating System

- **Raspberry Pi 5**: Ubuntu 24.04 LTS (64-bit)
- **Download**: https://ubuntu.com/download/raspberry-pi

### 2.2 Required Software

- ROS 2 Jazzy
- MAVROS
- PX4 Firmware (v1.14+ recommended)
- Gazebo Harmonic (for simulation testing)
- Python 3.12+

### 2.3 Python Packages

```bash
pip3 install numpy opencv-python pyyaml scipy
```

---

## 3. Quick Start

### 3.1 Automated Setup

Run the setup script on a fresh Ubuntu 24.04 installation:

```bash
# Clone the repository
cd ~
git clone https://github.com/kunalz06/DRONE.git drone

# Run the setup script
sudo ./drone/hardware/scripts/setup_raspberry_pi.sh

# Reboot
sudo reboot
```

### 3.2 Manual Setup (if automated fails)

```bash
# 1. Install ROS 2 Jazzy
sudo apt update && sudo apt install -y ros-jazzy-desktop

# 2. Source ROS
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 3. Install MAVROS
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-msgs

# 4. Install GeographicLib datasets
sudo apt install -y geographiclib-tools
sudo geographiclib-get-geoids egm2008-1

# 5. Add user to dialout group (for serial access)
sudo usermod -a -G dialout $USER

# 6. Clone repository
git clone https://github.com/kunalz06/DRONE.git ~/drone
```

---

## 4. Detailed Setup

### 4.1 Pixhawk Configuration

#### 4.1.1 Flash PX4 Firmware

1. Connect Pixhawk to computer via USB
2. Open QGroundControl
3. Navigate to **Vehicle Setup → Firmware**
4. Select **PX4 Flight Stack** (Standard)
5. Flash the firmware

#### 4.1.2 Required PX4 Parameters

Set these parameters in QGroundControl or via MAVLink:

```
# Airframe
SYS_AUTOSTART = 4001  # Quadrotor X configuration

# Serial Ports
SERIAL0_PROTOCOL = 2   # MAVLink 2 (USB)
SERIAL1_PROTOCOL = 2   # MAVLink 2 (TELEM1)
SERIAL1_BAUD = 57600   # 57600 baud for telemetry

# EKF2 Settings
EKF2_GPS_CHECK = 21    # GPS check flags
EKF2_AID_MASK = 1      # Use GPS for position

# Flight Modes
COM_RCL_EXCEPT = 4     # RTL on RC loss
COM_LOW_BAT_ACT = 2    # RTL on low battery

# Safety
COM_OBL_ACT = 2        # RTL on offboard loss
COM_OBL_RC_ACT = 0     # Disabled for autonomous

# Geofence (adjust as needed)
GF_ACTION = 4          # RTL on geofence breach
GF_MAX_HOR_DIST = 100  # Max horizontal distance (m)
GF_MAX_VER_DIST = 50   # Max vertical distance (m)
```

#### 4.1.3 Sensor Calibration

1. **Magnetometer**: Calibrate in QGroundControl (rotate drone in all directions)
2. **Accelerometer**: Calibrate in QGroundControl (place in 6 orientations)
3. **Gyroscope**: Calibrate in QGroundControl (keep drone still)
4. **RC Controller**: Calibrate and configure switches
5. **Flight Modes**: Configure mode switch for:
   - Position Mode (manual with GPS)
   - Offboard Mode (autonomous)
   - RTL Mode (return to launch)

### 4.2 Raspberry Pi Configuration

#### 4.2.1 Enable Serial Ports

Edit `/boot/firmware/config.txt`:

```bash
# Enable UART
enable_uart=1

# Disable Bluetooth (free up UART)
dtoverlay=disable-bt

# Enable SPI (for some sensors)
dtparam=spi=on

# Enable I2C (for some sensors)
dtparam=i2c_arm=on
```

#### 4.2.2 Configure USB Rules

Create `/etc/udev/rules.d/99-drone.rules`:

```bash
# Pixhawk
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", MODE="0666", SYMLINK+="pixhawk"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", SYMLINK+="pixhawk"

# RPLidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"

# USB Camera
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="*", MODE="0666"
```

Apply rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 4.2.3 Configure Network

For WiFi connection to base station, edit `/etc/netplan/99-drone-wifi.yaml`:

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
        "YourSSID":
          password: "YourPassword"
```

Apply:

```bash
sudo netplan apply
```

### 4.3 SSH Key Setup (for Data Sync)

```bash
# Generate SSH key on drone
ssh-keygen -t rsa -b 4096

# Copy to base station
ssh-copy-id base@192.168.1.1

# Test connection
ssh base@192.168.1.1 "echo Connection successful"
```

---

## 5. Configuration

### 5.1 Mission Parameters

Edit `config/mission_params.yaml` to customize mission behavior:

```yaml
mission:
  target_altitude: 4.0        # Mapping altitude (meters)
  control_rate: 20.0          # Control loop frequency (Hz)

velocity_limits:
  max_xy_speed: 1.45          # Max horizontal speed (m/s)
  max_z_speed: 0.6            # Max vertical speed (m/s)
  max_yaw_rate: 0.9           # Max yaw rate (rad/s)

mapping:
  arena_half_extent: 8.0      # Arena size (meters)
  lane_spacing: 0.9           # Sweep lane spacing (meters)

battery:
  low_warn_percent: 25.0      # Low battery warning
  critical_percent: 12.0      # Critical battery (forced RTL)

safety:
  max_distance_from_base: 15.0  # Geofence radius (meters)
  max_altitude: 10.0            # Max altitude (meters)
```

### 5.2 MAVROS Topics

Default MAVROS topic mappings:

| Drone Topic | MAVROS Topic | Description |
|-------------|--------------|-------------|
| `/odom` | `/mavros/local_position/odom` | Local position |
| `/cmd_vel` | `/mavros/setpoint_velocity/cmd_vel` | Velocity command |
| `/battery` | `/mavros/battery` | Battery state |

### 5.3 Hardware Topics

Configure sensor topics in `config/hardware_bridge.yaml`:

```yaml
sensors:
  lidar:
    topic: "/scan"
    frame: "lidar_link"
  
  front_camera:
    topic: "/front_camera/image_raw"
    frame: "front_camera_optical"
  
  down_camera:
    topic: "/down_camera/image_raw"
    frame: "down_camera_optical"
```

---

## 6. Testing

### 6.1 Pre-Flight Checklist

Before each flight:

- [ ] Battery charged (≥80%)
- [ ] Propellers secured and undamaged
- [ ] GPS lock (≥6 satellites, HDOP < 2)
- [ ] Compass calibrated
- [ ] RC controller connected
- [ ] Telemetry link established
- [ ] Safety area clear
- [ ] Flight mode switch functional

### 6.2 Ground Tests

#### 6.2.1 MAVROS Connection Test

```bash
# Terminal 1: Start MAVROS
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/pixhawk:57600

# Terminal 2: Check connection
ros2 topic echo /mavros/state

# Terminal 3: Check position
ros2 topic echo /mavros/local_position/odom
```

#### 6.2.2 Sensor Tests

```bash
# Check LiDAR
ros2 topic echo /scan

# Check cameras
ros2 topic echo /front_camera/image_raw --no-arr
ros2 topic echo /down_camera/image_raw --no-arr

# Check battery
ros2 topic echo /mavros/battery
```

#### 6.2.3 Motor Test (REMOVE PROPS!)

```bash
# Arm the vehicle (props removed!)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Test individual motors (short bursts only!)
ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.1}}" --once

# Disarm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

### 6.3 Flight Tests

#### 6.3.1 Manual Flight Test

1. Switch to Position Mode
2. Arm and take off manually
3. Verify stable hover
4. Test all directions
5. Test RTL function
6. Land and disarm

#### 6.3.2 Autonomous Test (Simulation First!)

```bash
# In simulation
cd ~/drone
HOME=/tmp gz sim -r worlds/arena.sdf

# In another terminal
ros2 launch launch/slam_arena_mission.launch.py
```

#### 6.3.3 Autonomous Test (Hardware - Controlled Environment)

```bash
# Start mission
cd ~/drone
python3 hardware/scripts/hardware_mission.py --mode hardware

# In another terminal, have manual control ready
python3 scripts/manual_control_link.py

# Press 'n' to return to autonomous, 'm' for manual, 'b' for RTL
```

---

## 7. Operation

### 7.1 Starting a Mission

#### Method 1: Manual Launch

```bash
# Terminal 1: MAVROS (if not using launch file)
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/pixhawk:57600

# Terminal 2: Mission
cd ~/drone
python3 hardware/scripts/hardware_mission.py --mode hardware --config config/mission_params.yaml
```

#### Method 2: Launch File

```bash
cd ~/drone
ros2 launch hardware/launch/hardware_mission.launch.py \
  fcu_url:=/dev/pixhawk:57600 \
  use_safety_manager:=true
```

#### Method 3: Systemd Service

```bash
sudo systemctl start drone-mission
sudo systemctl status drone-mission
```

### 7.2 Monitoring

#### Real-time Monitoring

```bash
# Dashboard (if display available)
# Will show automatically if DISPLAY is set

# Command line monitoring
ros2 topic echo /mavros/local_position/odom
ros2 topic echo /mavros/battery
ros2 topic echo /map --no-arr
```

#### Status Check Script

```bash
~/check_status.sh
```

### 7.3 Manual Override

During autonomous operation, manual override is always available:

```bash
python3 ~/drone/scripts/manual_control_link.py
```

Controls:
- `m` - Enable manual mode
- `n` - Return to autonomous
- `w/s` - Forward/backward
- `a/d` - Left/right
- `i/k` - Up/down
- `j/l` - Yaw left/right
- `space` - Stop
- `b` - Return to base
- `q` - Quit

### 7.4 Post-Mission

#### Data Sync

```bash
# Sync data to base station
~/drone/hardware/scripts/data_sync.sh

# With cleanup
~/drone/hardware/scripts/data_sync.sh --clean-after
```

#### Save Map

```bash
mkdir -p ~/drone/maps
ros2 run nav2_map_server map_saver_cli -f ~/drone/maps/mission_map
```

---

## 8. Troubleshooting

### 8.1 Common Issues

#### MAVROS Won't Connect

```bash
# Check serial port
ls -la /dev/pixhawk
ls -la /dev/ttyACM*

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Check MAVROS logs
ros2 topic echo /mavros/state
```

#### No GPS Lock

```bash
# Check GPS status
ros2 topic echo /mavros/global_position/raw/fix

# Check HDOP (should be < 2)
ros2 topic echo /mavros/global_position/gp_fix

# Move outdoors or use GPS simulator
```

#### Drone Won't Arm

Common causes:
1. GPS not locked
2. Compass not calibrated
3. RC not connected
4. Safety switch not pressed
5. Pre-arm checks failing

Check in QGroundControl for specific error.

#### Erratic Flight Behavior

1. Recalibrate compass (away from metal/EMI)
2. Check propeller orientation
3. Verify motor order
4. Check vibration levels

### 8.2 Log Analysis

```bash
# View mission log
tail -f ~/drone/logs/mission.log

# MAVROS diagnostic
ros2 topic echo /mavros/diagnostic

# System resources
htop
```

### 8.3 Emergency Procedures

1. **Loss of Control**: Press safety switch or switch to Manual/Position mode
2. **Flyaway**: Trigger RTL via RC or telemetry
3. **Low Battery**: Land immediately in safe area
4. **GPS Loss**: Switch to Stabilize mode and land manually
5. **Complete Failure**: Use parachute (if equipped)

---

## 9. Safety Guidelines

### 9.1 General Safety

- **Never fly alone** - Always have a spotter
- **Never fly over people** - Maintain safe distance
- **Never fly near airports** - Check airspace restrictions
- **Never fly in bad weather** - Wind < 15 km/h, no rain
- **Never exceed visual line of sight** unless authorized

### 9.2 Pre-Flight Safety

1. Check weather conditions
2. Verify airspace restrictions
3. Inspect all equipment
4. Confirm battery levels
5. Establish communication with team
6. Brief all participants

### 9.3 In-Flight Safety

1. Maintain situational awareness
2. Monitor battery level continuously
3. Watch for obstacles and other aircraft
4. Keep manual override ready
5. Have abort plan ready

### 9.4 Emergency Contacts

Keep these contacts accessible:

- Local air traffic control (if applicable)
- Emergency services
- Field operator
- Safety officer

---

## Appendix A: Quick Reference Commands

```bash
# Check system status
~/check_status.sh

# Start mission
cd ~/drone && python3 hardware/scripts/hardware_mission.py --mode hardware

# Manual control
python3 scripts/manual_control_link.py

# Data sync
~/drone/hardware/scripts/data_sync.sh

# Stop mission service
sudo systemctl stop drone-mission

# View logs
tail -f ~/drone/logs/mission.log

# Check topics
ros2 topic list
ros2 topic echo /mavros/state
```

---

## Appendix B: Configuration Files Reference

| File | Purpose |
|------|---------|
| `config/mission_params.yaml` | Main mission configuration |
| `config/slam_toolbox_arena.yaml` | SLAM parameters |
| `/etc/netplan/99-drone-wifi.yaml` | Network configuration |
| `/etc/systemd/system/drone-mission.service` | Auto-start service |

---

**Document Version:** 2.0.0  
**For assistance:** Check the GitHub repository issues page
