FROM 192.168.5.239:5000/robo-base:latest

#https://index.ros.org/p/cv_bridge/

RUN . /opt/ros/humble/setup.sh \
    && apt install -y \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control
