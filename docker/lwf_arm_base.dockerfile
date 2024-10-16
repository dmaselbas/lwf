FROM  arm64v8/ros:humble-ros-base-jammy
ARG ROS_DISTRO=humble

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID="0"
ENV ROS_VERSION="2"
ENV ROS_DISTRO=humble

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    ros-dev-tools \
    ros-humble-rmw-cyclonedds-cpp \
    python3-numpy \
    ros-humble-cv-bridge \
    python3-opencv \
    libboost-python-dev \
    ros-humble-camera-ros \
    ros-humble-cv-camera \
    ros-humble-image-transport \
    ros-humble-image-pipeline \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    wget git

WORKDIR /root/ros2_ws
COPY ros2_ws/src/ /root/ros2_ws/src/
RUN apt update && apt upgrade -y \
    && rosdep update \
    && rosdep install -y --ignore-src --from-paths src -r
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install
