FROM robo-base:latest

RUN . /opt/ros/humble/setup.sh \
    && apt install -y \
    python3-serial
