FROM robo-base:latest

RUN apt-get update && apt-get install -y \
    libqt5serialport5-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws/src

COPY ros2_ws/src ./
WORKDIR /root/ros2_ws
RUN colcon build --packages-select witmotion_ros

CMD source install/setup.bash && ros2 launch witmotion_ros wt61c.py
