FROM osrf/ros:humble-desktop-jammy

RUN apt update -y \
    && apt upgrade -y \
    && apt install -y \
    libasio-dev \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rclpy \
    ros-humble-rclcpp \
    ros-humble-rosidl-generator-cpp \
    ros-humble-rosidl-parser \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    ros-humble-imu-tools \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-rosdistro \
    python3-wstool \
    python3-pandas \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-diagnostic-updater \
    ros-humble-asio-cmake-module \
    ros-humble-ament* \
    ros-humble-librealsense2 \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-spatio-temporal-voxel-layer \
    libqt5serialport5-dev \
    ros-humble-foxglove-bridge \
    ros-humble-teleop-twist-joy \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-usb-cam \
    ros-humble-mimick-vendor \
    ros-humble-robot-localization \
    ros-humble-imu-filter-madgwick \
    ros-humble-depth-image-proc \
    ros-humble-libpointmatcher \
    ros-humble-libg2o \
    ros-humble-gtsam \
    ros-humble-mqtt-client

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID="0"
ENV ROS_VERSION="2"
ENV ROS_DISTRO=humble

COPY ros2_ws/src/ /root/ros2_ws/src/
RUN rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && rosdep update && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"
