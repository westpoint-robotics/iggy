cmake_minimum_required(VERSION 2.8.3)
project(structured_segmentation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  nodelet
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(cfg/GroundSegmenter.cfg)

  INCLUDE_DIRS include
  LIBRARIES structured_segmentation
  CATKIN_DEPENDS cv_bridge nodelet pcl_ros roscpp rospy sensor_msgs
  DEPENDS system_lib
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

## Declare a cpp library
add_library(structured_segmentation
   src/ground_segmenter.cpp
)

## Declare a cpp executable
#add_executable(structured_segmentation_node src/structured_segmentation_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(structured_segmentation_node structured_segmentation_generate_messages_cpp)
add_dependencies(structured_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(structured_segmentation
   ${catkin_LIBRARIES}
)

