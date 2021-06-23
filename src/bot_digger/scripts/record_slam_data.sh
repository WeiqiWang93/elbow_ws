#!/bin/sh
rosbag record -O nighttime /zed2/rgb/image_rect_color /zed2/depth/depth_registered /zed2/rgb/camera_info \
/zed1/rgb/image_rect_color /zed1/depth/depth_registered /zed1/rgb/camera_info \
/zed2/odom \
/imu1/data \
/imu2/data \
tf \
tf_static \