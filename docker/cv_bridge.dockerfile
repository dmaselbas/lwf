FROM 192.168.5.239:5000/robo-base:latest

#https://index.ros.org/p/cv_bridge/

RUN . /opt/ros/humble/setup.sh \
    && apt install -y \
    python3-numpy \
    ros-humble-cv-bridge \
    python3-opencv \
    libboost-python-dev
