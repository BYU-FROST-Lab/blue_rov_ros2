#!/bin/bash

# TODO setup parameters for the base scripts for directories and such
# TODO setup for use with docker containers

IP_ADDRESS="192.168.2.103"
FOLDER=""

# IP Address argument
while getopts "i:d:" opt; do
  case $opt in
    i) IP_ADDRESS="$OPTARG" ;;
    d) FOLDER="$OPTARG" ;;
    *) echo "Usage: $0 -i <IP_ADDRESS> -d <FOLDER>" >&2
       exit 1 ;;
  esac
done

# Require folder argument
if [ -z "$FOLDER" ]; then
  echo "Usage: $0 -i <IP_ADDRESS> -d <FOLDER>" >&2
  exit 1
fi

# Extract the last character of VEHICLE_ID and use it for IP_ADDRESS

echo "Using IP address: $IP_ADDRESS"

# Variables
REMOTE_USER="pi5"
REMOTE_FOLDER="/home/pi5/blue_rov_ros2/bags"
LOCAL_FOLDER="$HOME/blue_rov_ros2/bags/$FOLDER"

# TODO add a setup ssh script to setup ssh keys

# Create the local folder if it doesn't exist
mkdir -p "$LOCAL_FOLDER"

# Run rsync with sshpass to avoid password prompt
rsync -avzh --progress \
    "${REMOTE_USER}@${IP_ADDRESS}:${REMOTE_FOLDER}/" "$LOCAL_FOLDER/"

# # Check if rsync was successful
# if [ $? -eq 0 ]; then
#     printSuccess "Files copied successfully from ${VEHICLE_ID} to ${LOCAL_FOLDER}"
# else
#     printError "Failed to copy files from ${VEHICLE_ID}"
#     exit 1
# fi


# rsync -avPh pi5@192.168.2.103:/home/pi5/blue_rov_ros2/bags/* rc_tank/
