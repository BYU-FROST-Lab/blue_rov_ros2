# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS 2 (Humble) workspace for a BlueROV2 underwater vehicle. The system runs in two Docker containers:
- **`bluerov_ros2`** (on the vehicle): sensor drivers, MAVLink bridge, navigation processing
- **`bluerov_base`** (base station): visualization (Mapviz), ground control (Cockpit)

All ROS packages live in `packages/` which is volume-mounted into `/home/frostlab/ros2_ws/src` inside the container. Config files in `config/` are mounted to `/home/frostlab/config/`.

## Build & Run

All build/run commands are executed inside the container:

```bash
# Enter the vehicle container
docker exec -it bluerov_ros2 bash

# Build all packages
colcon build

# Build a single package
colcon build --packages-select sensor_bringup

# Source and launch
source install/setup.bash
ros2 launch sensor_bringup bluerov_launch.py

# Base station launch (in bluerov_base container)
ros2 launch sensor_bringup base_station_launch.py
```

### Useful shortcuts

```bash
# Full vehicle tmux session (from repo root, on vehicle)
bash scripts/tmux.sh

# Full base station tmux session (from repo root, at base station)
bash base_scripts/tmux.sh

# Run linting tests for a package
colcon test --packages-select sensor_bringup
colcon test-result --verbose
```

## Docker

```bash
# Vehicle container
cd docker && docker compose up -d

# Base station container (extends bluerov_ros2:latest with GUI packages)
cd docker/base_station && docker compose up -d
```

The base station Dockerfile (`docker/base_station/Dockerfile`) builds on top of `bluerov_ros2:latest` and adds Mapviz, RViz2, and X11 GUI support.

## Package Architecture

### `sensor_bringup`
Main orchestration package. Contains all launch files and several custom nodes:
- **`bluerov_launch.py`**: Vehicle-side launch — starts DVL, MAVLink bridge, dual pressure sensors, SBG IMU/GNSS, NTRIP client, NMEA/gpsd bridge
- **`base_station_launch.py`**: Base station — SBG driver, NTRIP client, NMEA bridge, Mapviz
- **`odom_anchor.py`**: One-shot anchoring node. Computes a fixed `map → odom_frame` TF using GPS + local odometry, gating on GPS covariance. Freezes the transform once initialized. Run multiple instances for different odometry sources (IMU, DVL).
- **`gps_odom.py`**: Converts `NavSatFix` → `nav_msgs/Odometry` using Haversine from a configurable origin. Fuses IMU orientation into the odometry pose.
- **`dvl_converter.py`**: Converts DVL messages to `TwistWithCovarianceStamped` and dead-reckoned `Odometry`. Applies NED→ENU coordinate transform.

### `mavlink_bridge`
Connects to ArduSub (BlueROV2 autopilot) via MAVLink UDP (`udpin:192.168.2.103:15550`). Publishes:
- `pressure/bar30` (`FluidPressure`) from `SCALED_PRESSURE2`
- `/mavros/rc/out` (`RCOut`) from `SERVO_OUTPUT_RAW`
- `battery/status` (`BatteryState`) from `BATTERY_STATUS`

### `dvl-a50`
Water Linked DVL-A50 driver. Connects to DVL at `192.168.2.95`. Publishes velocity and dead-reckoned position as custom `dvl_msgs/DVL` and `dvl_msgs/DVLDR` messages.

### `dvl-msgs`
Custom message definitions for the DVL-A50 (`DVL`, `DVLDR`).

### `pressure_sensor_ros2`
MS5837 pressure sensor driver. Launched as two separate namespaced instances (`/bluerov2/shallow` and `/bluerov2/deep`) for redundancy. Includes `pressure_pub` (reads I2C sensor) and `pressure_to_depth` (converts to depth with auto-calibration).

### `sbg_driver`
SBG Ellipse IMU/GNSS driver. Provides IMU data, magnetometer, GNSS fix, and EKF outputs. Config at `config/sbg_driver_params.yaml`.

### `sonar-3d-15`
Water Linked Sonar 3D-15 multibeam sonar driver. See `packages/sonar-3d-15/CLAUDE.md` for detailed architecture. Connects via UDP multicast to sonar at `192.168.194.96`.

### `time_sync_utils`
GPS-disciplined time synchronization. Bridges IMU NMEA output to `gpsd` over UDP so Chrony can use GPS as a time source. Full sync chain: `GPS → PPS → Chrony → PHC (phc2sys) → PTP (ptp4l) → sonar`.

## Configuration

All configs in `config/` are mounted into containers at `~/config/`:
- `vehicle_params.yaml`: Vehicle ID, GPS origin, per-node ROS parameters (pressure sensors, depth converter, odom anchors)
- `dvl_params.yaml`: DVL IP, frame IDs, TF parameters
- `sbg_driver_params.yaml`: SBG driver settings
- `ntrip_client_params.yaml`: RTK correction stream
- `time_sync_utils.yaml`: Topic monitor configuration
- `qos_record.yaml`: QoS overrides for bag recording

## Data Recording

Bags are recorded into `bags/` (mounted at `/home/frostlab/bags` in container):

```bash
# Interactive recording (prompts for bag name)
bash scripts/record.sh

# Record with a topic preset (e.g., IMU-only)
bash scripts/record.sh -p imu

# Monitor topic timestamps during recording
bash scripts/monitor_topics.sh
```

Topic presets are defined in `scripts/record_params.sh` (`TOPICS_REQ`, `TOPICS_EXTRA`, `TOPICS_IMU`). Storage format is MCAP with `fastwrite` preset.

## Network Layout

| Device | IP |
|---|---|
| Vehicle (BlueROV2 companion computer) | `192.168.2.103` |
| DVL-A50 | `192.168.2.95` |
| Sonar 3D-15 | `192.168.194.96` |
| ArduSub MAVLink (UDP) | `192.168.2.103:15550` |

## ROS Namespace

All vehicle nodes run under the `bluerov2` namespace (declared in launch files). Topic paths follow the pattern `/bluerov2/<subsystem>/<topic>`.
