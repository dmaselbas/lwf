FROM 192.168.5.239:5000/lwf:latest

#https://index.ros.org/p/camera_ros/
#https://index.ros.org/r/image_pipeline/

RUN . /opt/ros/humble/setup.sh \
    && apt update -y \
    && apt upgrade -y \
    && apt install -y \
    ros-humble-camera-ros \
    ros-humble-cv-camera \
    ros-humble-image-transport \
    ros-humble-image-pipeline
