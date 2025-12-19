# Multi-stage build to build some packages from source and install them
#  TODO


ARG ROS_DISTRO=humble 
FROM ros:${ROS_DISTRO}

# System dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-sbg-driver \
    ros-${ROS_DISTRO}-ouster-ros \
    ros-${ROS_DISTRO}-ntrip-client \
    && rm -rf /var/lib/apt/lists/*


# TODO remove below or just add to another dockerfile

# Additional helpful tools
RUN apt-get update && apt-get install -y \
    netcat \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /root/ros2_ws/src

# --- Source ROS2 ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
    
WORKDIR /root/ros2_ws
