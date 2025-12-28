# Time Synchronization Setup and Verification

This document describes how to configure GPS PPS on a Raspberry Pi and verify end-to-end time synchronization across the system, from GPS to Chrony, PHC, and PTP.

---

## Raspberry Pi PPS Configuration

Configure the PPS GPIO line by appending the following to `/boot/config.txt` on the Raspberry Pi:

```ini
# GPS PPS configuration
dtoverlay=pps-gpio,gpiopin=18
enable_uart=1
init_uart_baud=9600
```

Reboot the Raspberry Pi after making these changes.

---

## Time Synchronization Verification Steps

Follow the steps below in order to verify that time synchronization is functioning correctly.

---

### 1. Start IMU Driver and Acquire GPS Lock

Launch the sensor platform and wait until the IMU has a valid GPS lock and UTC time.

```bash
ros2 launch sensor_bringup sensor_platform_launch.py
```

---

### 2. Verify NMEA Topic Publishing

Confirm that NMEA messages are being published in ROS 2.

```bash
ros2 topic echo /nmea
```

---

### 3. Verify UDP NMEA Output and gpsd Input

Ensure the ROS node is subscribing to the NMEA topic and forwarding UDP NMEA strings (including constructed `GPRMC` sentences).

Check that `gpsd` is running and receiving raw NMEA data:

```bash
sudo systemctl status gpsd.service
gpspipe -r
```

---

### 4. Verify Chrony Is Using GPS Time

Confirm that `gpsd` is correctly feeding time to Chrony.

```bash
gpsmon
cgps
chronyc sources
```

Chrony should show GPS as a valid time source.

---

### 5. Verify PPS Discipline of Chrony

Check that the PPS signal is active and disciplining Chrony.

```bash
sudo ppstest /dev/pps0
chronyc sources
```

The PPS source should appear with low offset and high precision.

---

### 6. Verify PHC Discipline via phc2sys

Ensure that `phc2sys` is disciplining the Ethernet hardware clock using the system clock (already disciplined by Chrony).

```bash
sudo systemctl status phc2sys
sudo phc_ctl eth0 cmp
```

The offset between the system clock and the PHC should be minimal.

---

### 7. Verify PTP Distribution to External Devices

Confirm that `ptp4l` is running and disciplining downstream devices using the hardware clock.

```bash
sudo systemctl status ptp4l
curl -i -X GET http://<lidar_ip_address>/api/v1/time/ptp
```

The device should report an active PTP lock and synchronized time.

---

### 8. Start the Lidar Driver

```bash
ros2 launch sensor_bringup lidar_launch.py
```

### 9. Check the timestamps of the messages

```bash
bash scripts/monitor_topics.sh
```


## Summary

This setup establishes the following time chain:

```
GPS → PPS → Chrony (system clock) → PHC (phc2sys) → PTP (ptp4l) → External devices
```

If any step fails, verify that all upstream components are correctly synchronized before proceeding downstream.


# Recording Data

## Steps:

### 1. Mount the drive

```bash
bash scripts/mount_drive.sh
```

### 2. Record data

```bash
bash scripts/record.sh
```
Enter the name of the folder
This will copy the config folder into the bag once the recording is complete


### 3. Unmount the drive

```bash
bash scripts/mount_drive.sh -u
```