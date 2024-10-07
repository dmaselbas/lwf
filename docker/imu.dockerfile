FROM robo-base:latest

RUN apt-get update && apt-get install -y \
    libqt5serialport5-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws/src

COPY ros2_ws/src ./
