FROM debian:bullseye

# Install dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    wget \
    tar \
    python3-pip \
    python3-numpy \
    cmake \
    qemu-user-static \
    nano \
    git \
    xz-utils \
    && rm -rf /var/lib/apt/lists/*

# Get compiler
RUN wget -q --show-progress -O /tmp/cross-pi-gcc.tar.xz "https://rwth-aachen.sciebo.de/s/fvRliLwRTesg6Sl/download?path=%2F&files=cross-pi-gcc.tar.xz" \
    && tar xf /tmp/cross-pi-gcc.tar.xz -C /opt \
    && rm /tmp/cross-pi-gcc.tar.xz

# Get rootfs
RUN wget -q --show-progress -O /tmp/raspi-rootfs.tar.xz "https://rwth-aachen.sciebo.de/s/fvRliLwRTesg6Sl/download?path=%2F&files=raspi-rootfs.tar.xz" \
    && tar xf /tmp/raspi-rootfs.tar.xz -C /opt/  \
    && ln -s /opt/raspi-rootfs/usr/lib /opt/raspi-rootfs/lib  \
    && rm /tmp/raspi-rootfs.tar.xz

# Copy over toolchain
COPY docker_resources/raspi-toolchain.cmake /opt/raspi-toolchain.cmake

# Install ROS2 development dependencies
RUN pip3 install --no-cache-dir \
    colcon-common-extensions \
    vcstool \
    lark

# Add pi user
RUN useradd -m pi  \
    && echo "pi:pi" | chpasswd  \
    && usermod -aG sudo pi

# Switch to user
USER pi

# Create ROS2 workspace
RUN mkdir -p /home/pi/ros2_ws/src
WORKDIR /home/pi/ros2_ws

# Get ROS2 source code
RUN wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos && \
    vcs import /home/pi/ros2_ws/src < ros2.repos && \
    rm ros2.repos

# Disable not working packages
RUN touch ./src/eclipse-cyclonedds/COLCON_IGNORE \
./src/ros-visualization/COLCON_IGNORE \
./src/ros/ros_tutorials/turtlesim/COLCON_IGNORE \
./src/ros2/demos/image_tools/COLCON_IGNORE \
./src/ros2/demos/intra_process_demo/COLCON_IGNORE \
./src/ros2/demos/pendulum_control/COLCON_IGNORE \
./src/ros2/rviz/COLCON_IGNORE

# Set compiler
ENV C_INCLUDE_PATH="/opt/raspi-rootfs/usr/include:/opt/raspi-rootfs/usr/include/arm-linux-gnueabihf"
ENV CPLUS_INCLUDE_PATH="/opt/raspi-rootfs/usr/include:/opt/raspi-rootfs/usr/include/arm-linux-gnueabihf"

# Build ROS2
RUN colcon build \
--merge-install \
--cmake-force-configure \
--cmake-args \
-DCMAKE_TOOLCHAIN_FILE=/opt/raspi-toolchain.cmake \
-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
-DTHIRDPARTY=ON \
-DBUILD_TESTING:BOOL=OFF

# Copy over bashrc.sh
COPY docker_resources/bashrc.sh bashrc.sh
RUN cat bashrc.sh >> $HOME/.bashrc &&  \
    rm bashrc.sh

# Create vehicle workspace
RUN mkdir -p /home/pi/vehicle_ws/src
WORKDIR /home/pi/vehicle_ws/
