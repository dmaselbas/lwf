FROM  192.168.5.239:5000/lwf_arm_base:latest

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    ros-humble-imu-tools

COPY ros2_ws/src/witmotion_imu/ /root/ros2_ws/src/witmotion_imu/
ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch lwf_launch imu.launch.py"]
