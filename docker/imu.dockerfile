FROM  lwf_base

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    ros-humble-imu-tools

ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 launch lwf_launch imu.launch.py"]
