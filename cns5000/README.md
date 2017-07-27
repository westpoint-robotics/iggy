# CNS5000

#### Install cns5000 if using the KVH CNS500 IMU/GPS.
1. `cd ~/catkin_ws/src`
2. `git clone https://github.com/westpoint-robotics/iggy.git`
3. `sudo apt-get install ros-indigo-gps-common libpcap0.8-dev`
4. `sudo su`
5. `echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="2303", ATTRS{idVendor}=="067b", ATTRS{product}=="USB-Serial Controller", SYMLINK+="raw_imu", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-cns.rules`
6. `echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="2303", ATTRS{idVendor}=="067b", ATTRS{product}=="USB-Serial Controller D", SYMLINK+="raw_gps", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-cns.rules`
7. `echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="0100", ATTRS{idVendor}=="09d7", SYMLINK+="flex6_gps", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-cns.rules`
8. `udevadm control --reload-rules`
9. `exit`
10. `cd ~/catkin_ws`
11. `catkin_make`

#### How to prepare gps/imu node:

1. create/place gps_imu node pkg in catkin_ws
2. run roscore
3. run command: rosrun gps_imu usma_novatel_driver.py
4. look for rostopics: "fix" for gps data, "Pose2D" for imu velocity data
