FROM 192.168.5.239:5000/robo-base:latest

#https://index.ros.org/r/robot_state_publisher/github-ros-robot_state_publisher/
#https://index.ros.org/p/robot_state_publisher
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws/src

COPY ros2_ws/src ./
