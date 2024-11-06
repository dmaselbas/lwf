ARG ROS_DISTRO=humble
FROM lwf_base

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
   \
  wget

RUN apt update && apt upgrade -y \
     && apt install \
         ros-${ROS_DISTRO}-nav2* \
         ros-${ROS_DISTRO}-nav2-bringup \
         ros-${ROS_DISTRO}-navigation2
