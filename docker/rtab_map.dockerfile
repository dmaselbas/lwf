# Use the ROS 2 Humble base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libpcl-dev \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev \
    libopencv-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-pcl-ros \
    ros-humble-tf2-eigen \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-message-filters \
    ros-humble-interactive-markers \
    && rm -rf /var/lib/apt/lists/*

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    libopenni2-dev \
    libopenni-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /rtabmap_ws

# Clone RTAB-Map repositories
RUN git clone https://github.com/introlab/rtabmap.git src/rtabmap \
    && git clone -b ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros

# Build RTAB-Map core library
RUN . /opt/ros/humble/setup.sh && \
    cd src/rtabmap && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/humble -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install

# Build RTAB-Map ROS package
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select rtabmap_ros

# Source the ROS environment
ENV ROS_PACKAGE_PATH=/root/ros2_ws/src/rtabmap_ros:$ROS_PACKAGE_PATH

# Set the entrypoint
CMD ["bash", "-c", ". /opt/ros/humble/setup.bash && . install/setup.bash && ros2 launch rtabmap_ros rtabmap.launch.py"]
