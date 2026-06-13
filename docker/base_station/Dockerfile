FROM frostlab/bluerov_ros2:latest
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
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-diagnostic-aggregator \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    # x11-apps \
    # x11-utils \
    # x11-xserver-utils \
    # xauth \
    && rm -rf /var/lib/apt/lists/*

USER ${LABNAME}
WORKDIR /home/${LABNAME}/ros2_ws
