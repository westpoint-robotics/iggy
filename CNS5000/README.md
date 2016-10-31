# CNS5000

What am I....Please Fill in

##How to prepare gps/imu node:

1. create/place gps_imu node pkg in catkin_ws
2. run roscore
3. run command: rosrun gps_imu usma_novatel_driver.py
4. look for rostopics: "fix" for gps data, "Pose2D" for imu velocity data
