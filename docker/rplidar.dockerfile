ARG ROS_DISTRO=humble
FROM  arm64v8/ros:humble-ros-base-jammy

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
  ros-dev-tools \
  wget

# For Rolling or want to build from source a particular branch / fork
WORKDIR /root/ros2_ws
RUN mkdir -p ~/nav2_ws/src
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git ./src/rplidar_ros
RUN apt update && apt upgrade -y \
    && rosdep update \
    && rosdep install -y --ignore-src --from-paths src -r
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install

ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyS7"]
