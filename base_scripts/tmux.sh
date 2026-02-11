#!/bin/bash

SESSION_NAME=ros2
CONTAINER="bluerov_ros2"

# Start a new detached session
tmux new-session -d -s ${SESSION_NAME}

# Enable mouse support
tmux set-option -t ${SESSION_NAME} mouse on

# Enable color
# TODO do this in the config profile
tmux set-option -t ${SESSION_NAME} default-terminal "screen-256color"

########################
# Window 0: base
########################
tmux rename-window -t ${SESSION_NAME}:0 base

# Split into left/right (50/50)
tmux split-window -h -t ${SESSION_NAME}:0

# left pane
tmux send-keys -t ${SESSION_NAME}:0.0 \
  "cockpit" 
tmux send-keys -t ${SESSION_NAME}:0.1 \
  "clear" C-m
tmux send-keys -t ${SESSION_NAME}:0.1 \
  "bash base_scripts/sync_bags.sh -i 192.168.2.103 -d <FOLDER_NAME>"


########################
# Window 1: empty
########################
tmux new-window -t ${SESSION_NAME} -n monitor




# Select the first window
tmux select-window -t ${SESSION_NAME}:0
# Attach to session
tmux attach -t ${SESSION_NAME}
