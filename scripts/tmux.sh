#!/bin/bash

SESSION_NAME=ouster_sbg
CONTAINER="ouster_lidar_sbg"

# Start a new detached session
tmux new-session -d -s ${SESSION_NAME}

# Enable mouse support
tmux set-option -t ${SESSION_NAME} mouse on

########################
# Window 0: bringup
########################
tmux rename-window -t ${SESSION_NAME}:0 bringup

# Split into left/right (50/50)
tmux split-window -h -t ${SESSION_NAME}:0

# Split left pane into top/bottom
tmux split-window -v -t ${SESSION_NAME}:0.0

# Top-left pane
tmux send-keys -t ${SESSION_NAME}:0.0 \
  "docker exec -it ${CONTAINER} bash" C-m
tmux send-keys -t ${SESSION_NAME}:0.0 \
  "clear" C-m
tmux send-keys -t ${SESSION_NAME}:0.0 \
  "ros2 launch sensor_bringup sensor_platform_launch.py" 

# Bottom-left pane
tmux send-keys -t ${SESSION_NAME}:0.1 \
  "docker exec -it ${CONTAINER} bash" C-m
  tmux send-keys -t ${SESSION_NAME}:0.1 \
  "clear" C-m
tmux send-keys -t ${SESSION_NAME}:0.1 \
  "ros2 launch sensor_bringup lidar_launch.py" 

# Right pane
tmux send-keys -t ${SESSION_NAME}:0.2 \
  "docker exec -it ${CONTAINER} bash" C-m
tmux send-keys -t ${SESSION_NAME}:0.2 \
  "clear" C-m
tmux send-keys -t ${SESSION_NAME}:0.2 \
  "colcon build" 

########################
# Window 1: empty
########################
tmux new-window -t ${SESSION_NAME} -n monitor



########################
# Window 2: record
########################
tmux new-window -t ${SESSION_NAME} -n record
tmux send-keys -t ${SESSION_NAME}:record \
  "bash scripts/record.sh"

tmux split-window -h -t ${SESSION_NAME}:record
tmux send-keys -t ${SESSION_NAME}:record.1 \
  "bash scripts/record.sh -p sbg"


# Select the first window
tmux select-window -t ${SESSION_NAME}:0
# Attach to session
tmux attach -t ${SESSION_NAME}
