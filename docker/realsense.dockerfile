FROM 192.168.5.239:5000/robo_base:latest

RUN . /opt/ros/humble/setup.sh \
    && apt install -y \
    ros-humble-librealsense2* \
    ros-humble-realsense2*
