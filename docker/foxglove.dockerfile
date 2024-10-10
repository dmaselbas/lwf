FROM 192.168.5.239:5000/lwf:latest


RUN apt install

CMD ros2 launch foxglove_bridge foxflove_bridge_launch.xml
