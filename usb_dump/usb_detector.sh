#!/bin/bash

LOG_FILE="/tmp/usb_detector.log"
PYTHON_SCRIPT="/home/flip/PiLiDAR/usb_dump.py"

echo "$(date): Script started for device $1" >> "$LOG_FILE"

# Use udevadm to get device information
DEVICE_INFO=$(udevadm info --query=property --name="/dev/$1")

# Extract UUID and other relevant information
UUID=$(echo "$DEVICE_INFO" | grep ID_FS_UUID= | cut -d= -f2)
FS_TYPE=$(echo "$DEVICE_INFO" | grep ID_FS_TYPE= | cut -d= -f2)

echo "$(date): UUID: $UUID, Filesystem: $FS_TYPE" >> "$LOG_FILE"

if [ -z "$UUID" ]; then
    echo "$(date): Error - UUID not found for device $1" >> "$LOG_FILE"
    exit 1
fi

# Function to find the mount point
find_mount_point() {
    local uuid="$1"
    mount | grep -i "$uuid" | cut -d' ' -f3
}

# Wait for the mount point to become available
MAX_WAIT=30
for i in $(seq 1 $MAX_WAIT); do
    MOUNT_POINT=$(find_mount_point "$UUID")
    
    if [ -n "$MOUNT_POINT" ] && [ -d "$MOUNT_POINT" ]; then
        echo "$(date): Mount point $MOUNT_POINT found" >> "$LOG_FILE"
        
        # Execute the Python script
        echo "$(date): Executing Python script" >> "$LOG_FILE"
        /usr/bin/python3 "$PYTHON_SCRIPT" "$MOUNT_POINT" >> "$LOG_FILE" 2>&1
        
        echo "$(date): Python script execution completed" >> "$LOG_FILE"
        exit 0
    fi
    
    sleep 1
done

echo "$(date): Error - Mount point not found after $MAX_WAIT seconds" >> "$LOG_FILE"
echo "$(date): Final mount status:" >> "$LOG_FILE"
mount >> "$LOG_FILE"
exit 1