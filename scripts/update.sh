# !/bin/bash

set -e
cd "$(dirname "$0")/.." 
mkdir -p packages   

# TODO improve this for removing and re pulling with branches
# VCS import to update all repos
vcs import packages < .vcs/ros2_packages

cd packages/dvl-a50
git submodule update --init --recursive