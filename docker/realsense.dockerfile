FROM lwf_base:latest

ENTRYPOINT ["/bin/bash", "-c", "/ros_entrypoint.sh && source install/setup.bash && ros2 run realsense2_camera realsense2_camera_node --ros-args -p color_width:=640 -p color_height:=480 -p color_fps:=30 -p depth_width:=640 -p depth_height:=480 -p depth_fps:=30 -p enable_color:=true -p enable_depth:=true -p align_depth.enable:=true -p enable_sync:=true -p pointcloud.enable:=true"]
