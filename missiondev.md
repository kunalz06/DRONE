Autonomous Drone Project: Sequential Development Steps

This document outlines the step-by-step process for developing and executing the autonomous arena mission using a Raspberry Pi 5 and ROS 2 Jazzy.

Phase 1: Environment & Workspace Setup

Operating System Installation: Install Ubuntu 24.04 LTS (Noble Numbat) on the Raspberry Pi 5 and the development laptop.

ROS 2 Installation: Install ROS 2 Jazzy Jalisco on both machines.

Simulation Setup: Install Gazebo Harmonic (GZ Sim) on the development laptop.

Workspace Initialization: Create a ROS 2 workspace (drone_ws) and an ament_python package named arena_drone_system.

Dependency Management: Install necessary libraries: rclpy, sensor_msgs, geometry_msgs, nav_msgs, cv_bridge, mavros_msgs, and ros_gz_bridge.

Phase 2: Simulation & Model Development

Arena Creation: Build a 16m x 16m Gazebo world (arena.sdf) with yellow boundary markers and a central docking station.

Drone Modeling: Create a quadcopter SDF model with mass < 2kg, including a 360° LiDAR, an RGB camera, and an IMU.

Control Integration: Configure the MulticopterControl and OdometryPublisher Gazebo plugins.

Bridge Configuration: Establish the parameter_bridge to link Gazebo topics (/scan, /camera, /odom) to ROS 2.

Phase 3: FSM Logic Implementation

State 1: WAITING_FOR_CHARGE: Implement logic to monitor /mavros/battery. Block mission start until battery percentage $\geq 80\%$.

State 2: TAKEOFF: Develop a Proportional (P) controller to command the drone to climb and stabilize at a safety altitude of 3.0m.

State 3: MAPPING_SEARCH:

Integrate LiDAR data to rotate and avoid obstacles within 1.0m.

Use OpenCV HSV filtering to detect yellow lines and trigger boundary-reversal maneuvers.

Execute a search pattern to capture sample images.

State 4: RTL (Return to Launch): Create a failsafe triggered by 30% battery or a mission timer. Navigate back to the home origin $(0,0)$.

State 5: DOCKING:

Use downward-facing camera for ArUco marker detection.

Implement P-control alignment for $x$ and $y$ axes.

Initiate descent when pixel error is $< 20\text{px}$.

State 6: SYNCING: Detect landing ($z < 0.15\text{m}$) and trigger a background thread for data transfer.

Phase 4: Wireless Connectivity & Data Transfer

Network Configuration: Use nmcli to set a static IP on the Pi 5 and prioritize the Base Station Wi-Fi SSID.

SSH Authentication: Generate RSA keys on the Pi 5 and transfer the public key to the laptop via ssh-copy-id for passwordless access.

Autonomous Sync: Configure the rsync command with --remove-source-files to move data and clear drone storage after successful receipt.

Phase 5: Hardware Integration & Deployment

Hardware Assembly: Mount the LiDAR, Camera, and Pi 5 on the drone frame.

Flight Controller Link: Connect the Pi 5 to the Pixhawk via UART or USB and launch MAVROS.

Sensor Calibration: Calibrate the magnetometer and IMU to ensure stable localization without GPS.

Mission Launch: Execute the drone_mission.launch.py script.

Validation: Monitor the telemetry on the laptop to ensure the drone completes the 80%-to-30% autonomous cycle.
