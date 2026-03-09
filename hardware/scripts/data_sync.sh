#!/bin/bash
#
# Data Sync Script for Drone Post-Mission Data Transfer
# Automatically transfers mission data from drone to base station.
#
# This script is designed to run after landing to:
# - Sync maps, logs, and images to base station
# - Clean up old data on drone (optional)
# - Verify data integrity
#
# Usage:
#   ./data_sync.sh [--clean-after]
#
# Author: Drone Project Team
# Version: 2.0.0

set -e

# Configuration
DRONE_HOME="${HOME}/drone"
LOGS_DIR="${DRONE_HOME}/logs"
MAPS_DIR="${DRONE_HOME}/maps"
IMAGES_DIR="${DRONE_HOME}/images"
DATA_DIR="${DRONE_HOME}/data"

# Base station configuration
BASE_STATION_USER="base"
BASE_STATION_IP="192.168.1.1"
BASE_STATION_PATH="/data/drone_uploads"
BASE_STATION_SSH_PORT="22"

# Remote path (will create dated folder)
DATE_STAMP=$(date +%Y%m%d_%H%M%S)
REMOTE_PATH="${BASE_STATION_PATH}/${DATE_STAMP}"

# Flags
CLEAN_AFTER=false
DRY_RUN=false

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() { echo -e "${BLUE}[$(date +'%H:%M:%S')]${NC} $1"; }
success() { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --clean-after)
            CLEAN_AFTER=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --base-ip)
            BASE_STATION_IP="$2"
            shift 2
            ;;
        --base-user)
            BASE_STATION_USER="$2"
            shift 2
            ;;
        *)
            error "Unknown option: $1"
            exit 1
            ;;
    esac
done

log "=========================================="
log "Drone Data Sync Tool"
log "=========================================="
log "Source: ${DRONE_HOME}"
log "Destination: ${BASE_STATION_USER}@${BASE_STATION_IP}:${REMOTE_PATH}"
log "Clean after: ${CLEAN_AFTER}"
log "=========================================="

# Check SSH connectivity
log "Checking connectivity to base station..."
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes -p "${BASE_STATION_SSH_PORT}" \
    "${BASE_STATION_USER}@${BASE_STATION_IP}" "echo ok" &>/dev/null; then
    error "Cannot connect to base station via SSH"
    error "Check SSH key authentication and network connectivity"
    exit 1
fi
success "Base station reachable"

# Create remote directory
log "Creating remote directory..."
if [ "$DRY_RUN" = false ]; then
    ssh -p "${BASE_STATION_SSH_PORT}" "${BASE_STATION_USER}@${BASE_STATION_IP}" \
        "mkdir -p ${REMOTE_PATH}/{logs,maps,images,data}"
fi
success "Remote directory ready"

# Calculate local data size
TOTAL_SIZE=0
for dir in "$LOGS_DIR" "$MAPS_DIR" "$IMAGES_DIR" "$DATA_DIR"; do
    if [ -d "$dir" ]; then
        SIZE=$(du -sb "$dir" 2>/dev/null | cut -f1 || echo 0)
        TOTAL_SIZE=$((TOTAL_SIZE + SIZE))
    fi
done

TOTAL_SIZE_MB=$((TOTAL_SIZE / 1024 / 1024))
log "Total data to transfer: ${TOTAL_SIZE_MB} MB"

# Transfer function
transfer_dir() {
    local src_dir="$1"
    local dest_subdir="$2"
    
    if [ ! -d "$src_dir" ]; then
        warn "Directory not found: ${src_dir}"
        return 0
    fi
    
    local file_count=$(find "$src_dir" -type f 2>/dev/null | wc -l)
    if [ "$file_count" -eq 0 ]; then
        warn "No files in: ${src_dir}"
        return 0
    fi
    
    log "Transferring ${dest_subdir} (${file_count} files)..."
    
    if [ "$DRY_RUN" = true ]; then
        echo "  [DRY RUN] rsync -avz --progress ${src_dir}/ ${BASE_STATION_USER}@${BASE_STATION_IP}:${REMOTE_PATH}/${dest_subdir}/"
        return 0
    fi
    
    # Use rsync with progress and compression
    if rsync -avz --progress \
        -e "ssh -p ${BASE_STATION_SSH_PORT}" \
        --partial \
        --stats \
        "${src_dir}/" \
        "${BASE_STATION_USER}@${BASE_STATION_IP}:${REMOTE_PATH}/${dest_subdir}/"; then
        success "${dest_subdir} transferred"
        return 0
    else
        error "Failed to transfer ${dest_subdir}"
        return 1
    fi
}

# Transfer all data
TRANSFER_SUCCESS=true

transfer_dir "$LOGS_DIR" "logs" || TRANSFER_SUCCESS=false
transfer_dir "$MAPS_DIR" "maps" || TRANSFER_SUCCESS=false
transfer_dir "$IMAGES_DIR" "images" || TRANSFER_SUCCESS=false
transfer_dir "$DATA_DIR" "data" || TRANSFER_SUCCESS=false

# Create manifest
log "Creating manifest file..."
MANIFEST_FILE="/tmp/manifest_${DATE_STAMP}.txt"

cat > "$MANIFEST_FILE" << EOF
# Drone Mission Data Manifest
# Generated: $(date)
# Drone Hostname: $(hostname)

[Transfer Info]
Date: ${DATE_STAMP}
Total Size: ${TOTAL_SIZE_MB} MB
Success: ${TRANSFER_SUCCESS}

[System Info]
Uptime: $(uptime -p)
CPU Temp: $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | awk '{print $1/1000 " C"}' || echo "N/A")
Memory: $(free -h | grep Mem | awk '{print $3 "/" $2}')
Disk: $(df -h / | tail -1 | awk '{print $3 "/" $2}')

[Files]
EOF

# List all transferred files
for dir in "$LOGS_DIR" "$MAPS_DIR" "$IMAGES_DIR" "$DATA_DIR"; do
    if [ -d "$dir" ]; then
        echo "Directory: $dir" >> "$MANIFEST_FILE"
        find "$dir" -type f -exec ls -lh {} \; >> "$MANIFEST_FILE" 2>/dev/null
        echo "" >> "$MANIFEST_FILE"
    fi
done

# Transfer manifest
if [ "$DRY_RUN" = false ]; then
    rsync -avz -e "ssh -p ${BASE_STATION_SSH_PORT}" \
        "$MANIFEST_FILE" \
        "${BASE_STATION_USER}@${BASE_STATION_IP}:${REMOTE_PATH}/manifest.txt"
fi
rm -f "$MANIFEST_FILE"

success "Manifest transferred"

# Verify transfer
if [ "$DRY_RUN" = false ] && [ "$TRANSFER_SUCCESS" = true ]; then
    log "Verifying transfer..."
    
    # Check if remote directory has files
    REMOTE_FILES=$(ssh -p "${BASE_STATION_SSH_PORT}" \
        "${BASE_STATION_USER}@${BASE_STATION_IP}" \
        "find ${REMOTE_PATH} -type f | wc -l")
    
    if [ "$REMOTE_FILES" -gt 0 ]; then
        success "Verification complete: ${REMOTE_FILES} files on base station"
    else
        warn "No files found on base station - transfer may have failed"
        TRANSFER_SUCCESS=false
    fi
fi

# Clean up local data if requested
if [ "$CLEAN_AFTER" = true ] && [ "$TRANSFER_SUCCESS" = true ] && [ "$DRY_RUN" = false ]; then
    log "Cleaning up local data..."
    
    # Only clean if transfer succeeded
    read -p "This will DELETE local data. Confirm? (yes/no): " confirm
    if [ "$confirm" = "yes" ]; then
        rm -rf "${LOGS_DIR:?}"/*
        rm -rf "${IMAGES_DIR:?}"/*
        rm -rf "${DATA_DIR:?}"/*
        # Keep maps (they might be useful)
        success "Local data cleaned"
    else
        warn "Cleanup cancelled"
    fi
fi

# Summary
echo ""
log "=========================================="
if [ "$TRANSFER_SUCCESS" = true ]; then
    success "DATA SYNC COMPLETE"
    log "Remote location: ${BASE_STATION_USER}@${BASE_STATION_IP}:${REMOTE_PATH}"
else
    error "DATA SYNC HAD ERRORS"
    log "Some files may not have transferred correctly"
fi
log "=========================================="

exit $([ "$TRANSFER_SUCCESS" = true ] && echo 0 || echo 1)
