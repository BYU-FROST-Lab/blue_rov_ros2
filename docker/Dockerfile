ARG ROS_DISTRO=humble 
FROM ros:${ROS_DISTRO}

# TODO root vs frostlab user options for general container.

ARG LABNAME
ARG DEBIAN_FRONTEND=noninteractive
USER root

# Set up a new user
RUN useradd -ms /bin/bash ${LABNAME} \
    && usermod -aG sudo ${LABNAME} \
    && usermod -aG dialout ${LABNAME} \
    && echo "${LABNAME}:${LABNAME}" | chpasswd \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# System dependencies
RUN apt-get update && apt-get install -y \
    # ros-${ROS_DISTRO}-sbg-driver \
    ros-${ROS_DISTRO}-ntrip-client \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-mavros-msgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*


# Additional optional helpful debug tools
RUN apt-get update && apt-get install -y \
    netcat \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*


# Create i2c group with typical GID (if needed)
RUN groupadd -f -g 994 i2c && usermod -aG i2c ${LABNAME}

# # install LCM after libglib2.0-dev
# WORKDIR /home/frostlab
# RUN git clone https://github.com/lcm-proj/lcm.git
# WORKDIR /home/frostlab/lcm
# RUN mkdir build && cd build && \
#     cmake .. && \
#     make -j$(nproc) && make install && ldconfig

RUN mkdir -p /root/ros2_ws/src

# Dependency for i2c pressure sensors
RUN pip3 install \
    smbus2 \
    pymavlink \
    scipy \
    --no-cache-dir
    
USER ${LABNAME}


# --- Source ROS2 ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    
WORKDIR /home/${LABNAME}/ros2_ws
