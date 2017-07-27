# CNS5000

#### Setting for CNS-5000 IMU/GPS. 
Perform the following steps after you have cloned and compiled [iggy](https://github.com/westpoint-robotics/iggy.git)
1. `sudo apt-get install ros-indigo-gps-common libpcap0.8-dev`
2. `sudo su`
3. `echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="2303", ATTRS{idVendor}=="067b", ATTRS{product}=="USB-Serial Controller", SYMLINK+="raw_imu", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-cns.rules`
4. `echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="2303", ATTRS{idVendor}=="067b", ATTRS{product}=="USB-Serial Controller D", SYMLINK+="raw_gps", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-cns.rules`
5. `echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="0100", ATTRS{idVendor}=="09d7", SYMLINK+="flex6_gps", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-cns.rules`
6. `udevadm control --reload-rules`
7. `exit`
8. `cd ~/catkin_ws`
9. `catkin_make`

#### How to prepare gps/imu node:

1. create/place gps_imu node pkg in catkin_ws
2. run roscore
3. run command: rosrun gps_imu usma_novatel_driver.py
4. look for rostopics: "fix" for gps data, "Pose2D" for imu velocity data
