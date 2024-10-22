ros2 launch rtabmap_launch rtabmap.launch.py \
  frame_id:=base_link \
  database_path:=/opt/rtab/rtabmap.db \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  depth_camera_info_topic:=/camera/camera/depth/camera_info \
  approx_sync:=true \
  rgbd_sync:=true \
  rgbd_topic:=/camera/camera/rgbd \
  gen_cloud:=true \
  subscribe_scan:=true \




