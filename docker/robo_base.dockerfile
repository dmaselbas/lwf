FROM ros:humble-ros-base

RUN apt update -y \
    && apt upgrade -y \
    && apt install -y \
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
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-diagnostic-updater

WORKDIR /root/ros2_ws/src
COPY ros2_ws/src ./
WORKDIR /root/ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo ". /root/ros2_ws/install/setup.sh" >> /root/.bashrc
