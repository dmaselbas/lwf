FROM 192.168.5.239:5000/robo_base:latest

RUN apt-get update && apt-get install -y \
    libqt5serialport5-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws/src

COPY ros2_ws/src ./
WORKDIR /root/ros2_ws
RUN colcon build --packages-select witmotion_ros

CMD . install/setup.bash && ros2 launch witmotion_ros wt61c.py
