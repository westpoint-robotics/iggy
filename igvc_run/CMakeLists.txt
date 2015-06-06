cmake_minimum_required(VERSION 2.8.3)
project(igvc_run)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES igvc_roboteq
#  CATKIN_DEPENDS roscpp rospy serial
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(wheel_odom_tf_broadcaster src/wheel_odom_tf_broadcaster.cpp)
#target_link_libraries(wheel_odom_broadcaster ${catkin_LIBRARIES})

add_executable(wheel_odom_tf_broadcaster src/wheel_odom_tf_broadcaster.cpp)
target_link_libraries(wheel_odom_tf_broadcaster ${catkin_LIBRARIES})

