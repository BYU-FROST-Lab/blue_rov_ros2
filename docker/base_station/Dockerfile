FROM bluerov_ros2:latest
ARG LABNAME=frostlab
ARG ROS_DISTRO=humble 

ARG DEBIAN_FRONTEND=noninteractive

# Install graphical interface dependencies
USER root
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-mapviz \
    ros-${ROS_DISTRO}-mapviz-plugins \
    ros-${ROS_DISTRO}-tile-map \
    ros-${ROS_DISTRO}-multires-image \
    ros-${ROS_DISTRO}-rosapi \
    # x11-apps \
    # x11-utils \
    # x11-xserver-utils \
    # xauth \
    # ros-${ROS_DISTRO}-rqt* \
    && rm -rf /var/lib/apt/lists/*

USER ${LABNAME}
WORKDIR /home/${LABNAME}/ros2_ws
