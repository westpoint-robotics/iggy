# USMA IGVC Sterevision

### Dependancies
#### Desktop
* Ubuntu 14.04
* CUDA toolkit 7.5
* ZED SDK 1.1.0
* OpenCV 3.1 
#### Jetson
* Ubuntu 14.04
* CUDA toolkit 7.5 (automatic)
* ZED SDK 1.1.1a
* OpenCV 2.4

### Installation Instructions
#### Install CUDA (7.5)
1. sudo dpkg -i cuda-repo-ubuntu1404-7-5-local_7.5-18_amd64.deb
2. sudo apt-get update
3. sudo apt-get install cuda

#### Install OpenCV
1. unzip opencv-3.1*
2. cd opencv-3.1*
3. mkdir release
4. cd release
5. cmake ..
6. make -j8 -l8
7. sudo make install

#### Install ZED SDK
1. Get TK1 version of ZED SKD from:
  * https://www.stereolabs.com/developers/release/1.1.0/
2. cd {Download locaiton}
3. chmod 777 ZED*
4. ./ZED

#### Install ROS
1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
2. sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
3. sudo apt-get update
4. sudo apt-get install ros-indigo-ros-base
5. sudo apt-get install python-rosdep
6. sudo rosdep init
7. rosdep update
8. echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
9. source ~/.bashrc
10. sudo apt-get install python-rosinstall

#### Install Rvz
1. sudo apt-get install ros-indigo-rviz
2. sudo apt-get install ros-indigo-robot-model
3. echo "unset GTK_IM_MODULE" >> ~/.bashrc
4. source ~/.bashrc

#### Install pcl
1. sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
2. sudo apt-get update
3. sudo apt-get install libpcl-all

#### Install pcl-ros
1. sudo apt-get install ros-indigo-pcl-ros

#### Install zed-ros-wrapper
1. cd ~/[CATKIN_WOKRSPACE]/src
2. git clone https://github.com/stereolabs/zed-ros-wrapper 
3. cd ~/[CATKIN_WORKSPACE]
4. catkin_make zed-ros-wrapper
5. source ./devel/setup.bash

#### Recompile OpenCV
1. cd ~/[CATKIN_WORKSPACE]/src
2. git clone https://github.com/ros-perception/vision_opencv
3. cd ..
4. catkin_make --pkg vision_opencv

### ROS Networking
#### ON JETSON TK1:	
##### Jetson IP = 192.168.2.201
##### Remote IP = 192.168.2.200
1. echo export ROS_MASTER_URI=http://[REMOTE_PC_IP]:11311 >> ~/.bashrc
2. echo export ROS_HOSTNAME=[JETSON_IP] >> ~/.bashrc
3. echo export ROS_MASTER_URI=http://[REMOTE_PC_IP]:11311 >> ~/[CATKIN_WORKSPACE]/devel/setup.sh
4. echo export ROS_HOSTNAME=[JETSON_IP] >> ~/[CATKIN_WORKSPACE]/devel/setup.sh
5. export ROS_IP=[JETSON_IP]

#### ON REMOTE PC
1. echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc
2. echo export ROS_HOSTNAME=[PC_IP] >> ~/.bashrc
3. echo export ROSLAUNCH_SSH_UNKNOWN = 1 >> ~/.bashrc
