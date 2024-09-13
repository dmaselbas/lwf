# Use the official ROS 2 Humble base image
FROM ros:humble

# Install Python3 and pip if not already included
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip install poetry
# Create a workspace for your ROS2 project
WORKDIR /ros2_ws

# Copy your Python dependencies (if you have a requirements.txt)
COPY requirements.txt /ros2_ws/

# Install Python dependencies
RUN pip install -r requirements.txt

# Setup ROS 2 environment
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"

# Set the default command to bash
CMD ["bash"]
