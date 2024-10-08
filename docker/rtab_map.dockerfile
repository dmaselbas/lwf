FROM introlab3it/rtabmap_ros:humble-latest

RUN add-apt-repository universe --yes
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list
RUN apt install ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp -y
