FROM robo-base:latest

RUN . /opt/ros/humble/setup.sh \
    && apt install -y \
    ros-humble-librealsense2* \
    librealsense2-dkms \
    librealsense2-utils \
    ros-humble-realsense2-camera \
