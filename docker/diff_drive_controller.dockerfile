FROM robo-base:latest

#https://index.ros.org/p/cv_bridge/

RUN source /opt/ros/humble/setup.bash \
    && apt install -y \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control
