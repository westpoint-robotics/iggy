#!/bin/bash

rosbag record -O $HOME/catkin_ws/rosbags/imudata.bag /imu/data &
rosbag record -O $HOME/catkin_ws/rosbags/gps_fix.bag /gps/fix &
#rosbag record -O $HOME/catkin_ws/rosbags/enc_raw.bag /enc_raw &
rosbag record -O $HOME/catkin_ws/rosbags/pointcloud.bag /velodyne_points &
rosbag record -O $HOME/catkin_ws/rosbags/right_camera.bag /camera/right/rgb &
rosbag record -O $HOME/catkin_ws/rosbags/left_camera.bag /camera/left/rgb &
rosbag record -O $HOME/catkin_ws/rosbags/raw_gpsImu.bag /raw_data
