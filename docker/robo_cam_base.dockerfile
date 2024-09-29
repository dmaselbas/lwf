FROM robo-base:latest

#https://index.ros.org/p/camera_ros/
#https://index.ros.org/r/image_pipeline/

RUN source /opt/ros/humble/setup.bash \
    && apt update -y \
    && apt upgrade -y \
    && apt install -y \
    ros-humble-camera-ros \
    ros-humble-cv-camera \
    ros-humble-image-transport \
    ros-humble-image-pipeline
