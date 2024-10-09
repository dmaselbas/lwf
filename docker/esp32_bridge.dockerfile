FROM 192.168.5.239:5000/lwf:latest

RUN . /opt/ros/humble/setup.sh \
    && apt install -y \
    python3-serial
