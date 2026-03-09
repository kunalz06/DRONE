#!/bin/bash
#
# Raspberry Pi 5 Setup Script for Drone Project
# This script sets up the complete environment on a Raspberry Pi 5
# running Ubuntu 24.04 for hardware deployment.
#
# Usage:
#   chmod +x setup_raspberry_pi.sh
#   sudo ./setup_raspberry_pi.sh
#
# Author: Drone Project Team
# Version: 2.0.0

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Log function
log() {
    echo -e "${BLUE}[$(date +'%Y-%m-%d %H:%M:%S')]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    log_error "Please run as root (use sudo)"
    exit 1
fi

# Check Ubuntu version
if [ ! -f /etc/os-release ]; then
    log_error "Cannot detect OS version"
    exit 1
fi

source /etc/os-release
if [ "$VERSION_ID" != "24.04" ]; then
    log_warn "This script is designed for Ubuntu 24.04. You have $VERSION_ID"
    read -p "Continue anyway? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Configuration
DRONE_USER="${SUDO_USER:-pi}"
DRONE_HOME=$(eval echo "~$DRONE_USER")
DRONE_REPO="$DRONE_HOME/drone"
WIFI_SSID="DroneBase"
WIFI_PASSWORD="drone1234"
STATIC_IP="192.168.1.100"

log "=============================================="
log "Raspberry Pi 5 Drone Setup Script"
log "=============================================="
log "User: $DRONE_USER"
log "Home: $DRONE_HOME"
log "Repository: $DRONE_REPO"
log "=============================================="

# Step 1: System Update
log "Step 1: Updating system packages..."
apt update && apt upgrade -y
log_success "System updated"

# Step 2: Install ROS 2 Jazzy
log "Step 2: Installing ROS 2 Jazzy..."

# Add ROS 2 repository
apt install -y software-properties-common curl
add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update
apt install -y ros-jazzy-desktop ros-jazzy-ros-base

# Add ROS sourcing to bashrc
if ! grep -q "source /opt/ros/jazzy/setup.bash" "$DRONE_HOME/.bashrc"; then
    echo "" >> "$DRONE_HOME/.bashrc"
    echo "# ROS 2 Jazzy" >> "$DRONE_HOME/.bashrc"
    echo "source /opt/ros/jazzy/setup.bash" >> "$DRONE_HOME/.bashrc"
    chown "$DRONE_USER:$DRONE_USER" "$DRONE_HOME/.bashrc"
fi

log_success "ROS 2 Jazzy installed"

# Step 3: Install MAVROS and dependencies
log "Step 3: Installing MAVROS and drone dependencies..."

apt install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server \
    ros-jazzy-cv-bridge \
    ros-jazzy-actuator-msgs \
    ros-jazzy-geographic-msgs \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    python3-serial \
    i2c-tools \
    spi-tools

# Install GeographicLib datasets for MAVROS
log "Installing GeographicLib datasets (this may take a while)..."
apt install -y geographiclib-tools
geographiclib-get-geoids egm2008-1
geographiclib-get-gravity egm2008
geographiclib-get-magnetic emm2015

log_success "MAVROS and dependencies installed"

# Step 4: Install Gazebo (optional, for simulation testing)
log "Step 4: Installing Gazebo Harmonic (optional for simulation)..."

# Check if we have enough space
AVAILABLE_SPACE=$(df -BG / | tail -1 | awk '{print $4}' | tr -d 'G')
if [ "$AVAILABLE_SPACE" -gt 15 ]; then
    apt install -y gz-harmonic
    apt install -y ros-jazzy-ros-gz-bridge
    log_success "Gazebo Harmonic installed"
else
    log_warn "Skipping Gazebo installation (low disk space: ${AVAILABLE_SPACE}GB)"
fi

# Step 5: Configure Serial Ports
log "Step 5: Configuring serial ports for Pixhawk..."

# Add user to dialout group for serial access
usermod -a -G dialout "$DRONE_USER"

# Configure serial ports
cat > /etc/udev/rules.d/99-pixhawk.rules << 'EOF'
# Pixhawk/PX4 Flight Controller
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0010", SYMLINK+="pixhawk", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="pixhawk", MODE="0666"

# FTDI Serial (common for telemetry)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="telemetry", MODE="0666"

# Generic USB-Serial
SUBSYSTEM=="tty", ATTRS{product}=="USB-Serial", SYMLINK+="usb_serial", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger

log_success "Serial ports configured"

# Step 6: Configure WiFi
log "Step 6: Configuring WiFi..."

# Create netplan configuration for static IP
cat > /etc/netplan/99-drone-wifi.yaml << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: no
      addresses:
        - ${STATIC_IP}/24
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4
      access-points:
        "${WIFI_SSID}":
          password: "${WIFI_PASSWORD}"
EOF

# Set correct permissions
chmod 600 /etc/netplan/99-drone-wifi.yaml

log_warn "WiFi configured with static IP: $STATIC_IP"
log_warn "SSID: $WIFI_SSID (change these in /etc/netplan/99-drone-wifi.yaml)"

# Step 7: Configure SSH
log "Step 7: Configuring SSH..."

apt install -y openssh-server
systemctl enable ssh
systemctl start ssh

# Generate SSH keys if not exist
if [ ! -f "$DRONE_HOME/.ssh/id_rsa" ]; then
    sudo -u "$DRONE_USER" ssh-keygen -t rsa -b 4096 -f "$DRONE_HOME/.ssh/id_rsa" -N ""
    log_success "SSH key generated at $DRONE_HOME/.ssh/id_rsa.pub"
fi

# Step 8: Create workspace and clone repository
log "Step 8: Setting up drone repository..."

if [ ! -d "$DRONE_REPO" ]; then
    sudo -u "$DRONE_USER" git clone https://github.com/kunalz06/DRONE.git "$DRONE_REPO"
    log_success "Repository cloned"
else
    log_warn "Repository already exists, updating..."
    cd "$DRONE_REPO"
    sudo -u "$DRONE_USER" git pull
fi

# Create workspace directory structure
sudo -u "$DRONE_USER" mkdir -p "$DRONE_HOME/drone_ws/src"

# Step 9: Configure system limits
log "Step 9: Configuring system limits..."

# Increase file descriptor limits
cat > /etc/security/limits.d/drone.conf << EOF
# Drone mission limits
$DRONE_USER soft nofile 65536
$DRONE_USER hard nofile 65536
$DRONE_USER soft memlock unlimited
$DRONE_USER hard memlock unlimited
EOF

# Disable swap for better real-time performance
if [ -f /etc/dphys-swapfile ]; then
    sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=0/' /etc/dphys-swapfile
    systemctl restart dphys-swapfile 2>/dev/null || true
fi

# Step 10: Create systemd service for auto-start
log "Step 10: Creating systemd service..."

cat > /etc/systemd/system/drone-mission.service << EOF
[Unit]
Description=Drone Mission Service
After=network.target
Wants=network.target

[Service]
Type=simple
User=$DRONE_USER
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && cd $DRONE_REPO && python3 hardware/scripts/hardware_mission.py --mode hardware"
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable drone-mission.service

log_success "Systemd service created (not started)"

# Step 11: Install monitoring tools
log "Step 11: Installing monitoring and diagnostic tools..."

apt install -y \
    htop \
    iotop \
    nethogs \
    lm-sensors \
    stress \
    screen \
    tmux

# Step 12: Create helper scripts
log "Step 12: Creating helper scripts..."

# Start mission script
cat > "$DRONE_HOME/start_mission.sh" << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd "$HOME/drone"
python3 hardware/scripts/hardware_mission.py --mode hardware "$@"
EOF
chmod +x "$DRONE_HOME/start_mission.sh"
chown "$DRONE_USER:$DRONE_USER" "$DRONE_HOME/start_mission.sh"

# Check status script
cat > "$DRONE_HOME/check_status.sh" << 'EOF'
#!/bin/bash
echo "=== Drone System Status ==="
echo ""
echo "CPU Temperature:"
cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | awk '{print $1/1000 "°C"}' || echo "N/A"
echo ""
echo "Memory Usage:"
free -h
echo ""
echo "Disk Usage:"
df -h /
echo ""
echo "ROS Topics:"
source /opt/ros/jazzy/setup.bash
ros2 topic list 2>/dev/null || echo "ROS not running"
echo ""
echo "MAVROS Status:"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}" 2>/dev/null || echo "MAVROS not connected"
EOF
chmod +x "$DRONE_HOME/check_status.sh"
chown "$DRONE_USER:$DRONE_USER" "$DRONE_HOME/check_status.sh"

# Backup script
cat > "$DRONE_HOME/backup_data.sh" << 'EOF'
#!/bin/bash
BACKUP_DIR="$HOME/drone_backups"
DATE=$(date +%Y%m%d_%H%M%S)
mkdir -p "$BACKUP_DIR"

# Backup maps
if [ -d "$HOME/drone/maps" ]; then
    cp -r "$HOME/drone/maps" "$BACKUP_DIR/maps_$DATE"
fi

# Backup logs
if [ -d "$HOME/drone/logs" ]; then
    cp -r "$HOME/drone/logs" "$BACKUP_DIR/logs_$DATE"
fi

echo "Backup created at $BACKUP_DIR"
EOF
chmod +x "$DRONE_HOME/backup_data.sh"
chown "$DRONE_USER:$DRONE_USER" "$DRONE_HOME/backup_data.sh"

# Step 13: Set up log directory
log "Step 13: Setting up log directory..."

mkdir -p "$DRONE_REPO/logs"
chown -R "$DRONE_USER:$DRONE_USER" "$DRONE_REPO/logs"

# Step 14: Performance tuning
log "Step 14: Performance tuning..."

# Set CPU governor to performance
echo "performance" | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null || true

# Increase UDP buffer sizes
cat >> /etc/sysctl.conf << EOF

# Drone network tuning
net.core.rmem_max = 26214400
net.core.rmem_default = 1048576
net.core.wmem_max = 26214400
net.core.wmem_default = 1048576
net.ipv4.udp_mem = 65536 131072 26214400
EOF

sysctl -p

# Step 15: Final setup
log "Step 15: Final setup..."

# Create README for the user
cat > "$DRONE_HOME/README_DRONE_SETUP.txt" << EOF
================================
Drone Setup Complete
================================

Your Raspberry Pi 5 has been configured for autonomous drone operations.

Quick Start:
1. Connect Pixhawk to USB (will appear as /dev/pixhawk)
2. Power on sensors (LiDAR, cameras)
3. Run: ~/start_mission.sh

Useful Commands:
- Check status: ~/check_status.sh
- View logs: tail -f ~/drone/logs/mission.log
- Start mission service: sudo systemctl start drone-mission
- Stop mission service: sudo systemctl stop drone-mission
- Manual control: python3 ~/drone/scripts/manual_control_link.py

Configuration Files:
- Mission params: ~/drone/config/mission_params.yaml
- WiFi config: /etc/netplan/99-drone-wifi.yaml
- Service file: /etc/systemd/system/drone-mission.service

Static IP: $STATIC_IP
WiFi SSID: $WIFI_SSID

Important:
- Change WiFi credentials in /etc/netplan/99-drone-wifi.yaml
- Apply WiFi changes: sudo netplan apply
- Your SSH public key: ~/.ssh/id_rsa.pub

For issues, check logs at: ~/drone/logs/
================================
EOF

chown "$DRONE_USER:$DRONE_USER" "$DRONE_HOME/README_DRONE_SETUP.txt"

# Summary
log "=============================================="
log_success "SETUP COMPLETE!"
log "=============================================="
echo ""
echo "Next steps:"
echo "1. Reboot the system: sudo reboot"
echo "2. Connect your Pixhawk flight controller"
echo "3. Connect LiDAR and cameras"
echo "4. Read: $DRONE_HOME/README_DRONE_SETUP.txt"
echo ""
echo "To start a mission:"
echo "  cd ~/drone && python3 hardware/scripts/hardware_mission.py --mode hardware"
echo ""
log "=============================================="
