#!/bin/bash

# TODO make it dynamic which container to exec into

docker exec -it ouster_lidar_sbg bash -c \
  "source /root/ros2_ws/install/setup.bash && \
   ros2 run topic_monitor topic_monitor_node \
   --ros-args --params-file /root/config/time_sync_utils.yaml"