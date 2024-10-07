FROM robo-base:latest

RUN . /opt/ros/humble/setup.bash \
    && apt install -y \
    python3-serial
