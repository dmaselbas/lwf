FROM robo-base:latest

RUN source /opt/ros/humble/setup.bash \
    && apt install -y \
    ros-humble-librealsense2* \
    librealsense2-dkms \
    librealsense2-utils \
    ros-humble-realsense2-camera \
