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
    python3-xdiagnose \
    python3-xacro \
    python3-rosbag2 \
    python3-rosbag2-bagfile-v2 \
    python3-rosbag2-converter \
    python3-rosbag2-storage \
    python3-rosbag2-typesupport \
    python3-rosbag2-compression \
    python3-rosbag2-diagnostics \
    python3-rosbag2-py3-common \
    python3-rosbag2-python \
    python3-rosbag2-rosbag2-storage \
    python3-rosbag2-storage-default-plugins \
    python3-rosbag2-transport \
    python3-rosbag2-transport-default-plugins \
    python3-rosbag2-validate \
    python3-rosbag2-yaml \
    python3-rosbag2-dataset \
    python3-rosbag2-interface \
    python3-rosbag2-storage-all \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-diagnostic-updater


RUN . /opt/ros/humble/setup.bash \
    && rosdep init \
    && rosdep update

WORKDIR /root/ros2_ws/src
COPY ros2_ws/src ./
WORKDIR /root/ros2_ws
RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo ". /root/ros2_ws/install/setup.bash" >> /root/.bashrc
