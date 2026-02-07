#!/bin/bash
# --------------------------------------------------
# Rosbag recording parameters
# This file is sourced by record_bag.sh
# --------------------------------------------------

# -----------------------------
# Container / paths
# -----------------------------
CONTAINER="bluerov_ros2"
BAGS_PATH="/home/frostlab/bags"

# -----------------------------
# Rosbag storage settings
# -----------------------------
STORAGE="mcap"
PRESET="fastwrite"

# 0 means “no duration limit” unless overridden
DURATION=120

# Bytes (example: 200 MB)
MAX_CACHE_SIZE=200000000

# -----------------------------
# Topic presets
# -----------------------------
ALL="-a"

TOPICS_REQ="/imu/data /imu/mag /dvl/twist /dvl/data /dvl/position /deep/depth_data /deep/pressure/data /shallow/pressure/data /shallow/depth_data /imu/nav_sat_fix /tf /tf_static"

TOPICS_EXTRA=" \
/sbg/gps_pos \
/sbg/ekf_quat \
/sbg/ekf_nav \
/sbg/imu_data \
/sbg/imu_short \
/sbg/utc_time \
/sbg/mag \
/imu/utc_ref \
/imu/pos_ecef \
/imu/velocity \
/imu/mag \
/sbg/mag \
/nmea \
/rtcm \
/tf_static"

TOPICS_IMU=" \
/bluerov2/sbg/ekf_quat \
/bluerov2/sbg/ekf_nav \
/bluerov2/sbg/imu_data \
/bluerov2/sbg/imu_short \
/bluerov2/sbg/utc_time \
/bluerov2/sbg/mag \
/bluerov2/imu/data \
/bluerov2/imu/mag \
/bluerov2/sbg/mag \
"

# -----------------------------
# Default topics (used if no -p or -t)
# -----------------------------
TOPICS="${ALL}"
