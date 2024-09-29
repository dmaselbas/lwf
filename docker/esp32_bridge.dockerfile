FROM robo-base:latest

RUN source /opt/ros/humble/setup.bash \
    && apt install -y \
    python3-serial
