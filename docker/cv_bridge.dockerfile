FROM robo-base:latest

#https://index.ros.org/p/cv_bridge/

RUN source /opt/ros/humble/setup.bash \
    && apt install -y \
    python3-numpy \
    ros-humble-cv-bridge \
    python3-opencv \
    libboost-python-dev
