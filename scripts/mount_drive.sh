#!/bin/bash

# Default values
MOUNT_POINT="/home/pi5/sensor_platform_ros2/bags/ssd"
DEVICE="/dev/sda1"
ACTION="mount"

# Parse options
while getopts "t:u" opt; do
    case $opt in
        t)
            DEVICE="$OPTARG"
            ;;
        u)
            ACTION="umount"
            ;;
        *)
            echo "Usage: $0 [-t /dev/sdX] [-u]"
            echo "  -t  target device (default /dev/sda1)"
            echo "  -u  unmount whatever is mounted at $MOUNT_POINT"
            exit 1
            ;;
    esac
done

mkdir -p "$MOUNT_POINT"

case "$ACTION" in
    mount)
        echo "Mounting SSD $DEVICE to $MOUNT_POINT..."
        sudo mount "$DEVICE" "$MOUNT_POINT"
        if [ $? -eq 0 ]; then
            echo "SSD mounted successfully."
        else
            echo "Failed to mount SSD."
        fi
        ;;
    umount)
        echo "Unmounting SSD from $MOUNT_POINT..."
        sudo umount "$MOUNT_POINT"
        if [ $? -eq 0 ]; then
            echo "SSD unmounted successfully."
        else
            echo "Failed to unmount SSD."
        fi
        ;;
esac
