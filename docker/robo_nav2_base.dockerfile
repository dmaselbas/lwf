ARG ROS_DISTRO=humble
FROM 192.168.5.239:5000/lwf:latest

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
   \
  wget

RUN . /opt/ros/humble/setup.sh \
    && rosdep init

RUN apt update && apt upgrade -y \
     && rosdep update \
     && apt install \
         ros-${ROS_DISTRO}-nav2* \
         ros-${ROS_DISTRO}-nav2-bringup \
         ros-${ROS_DISTRO}-navigation2
