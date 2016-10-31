cmake_minimum_required(VERSION 2.8.3)
project(roboteq)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  tf
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES roboteq
#  CATKIN_DEPENDS roscpp rospy serial
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
#add_subdirectory(mbs)

## Declare a cpp library
# add_library(roboteq
#   src/${PROJECT_NAME}/roboteq.cpp
# )

## Declare a cpp executable
add_executable(roboteq_odom_node src/oop_igvc_odometry.cpp)
add_executable(wheel_odom_broadcaster src/wheel_odom_broadcaster.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(roboteq_node roboteq_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(roboteq_odom_node
   ${catkin_LIBRARIES}
 )

 target_link_libraries(wheel_odom_broadcaster
   ${catkin_LIBRARIES}
 )

