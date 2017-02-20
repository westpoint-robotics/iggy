cmake_minimum_required(VERSION 2.8.3)
project(gazebo_iggy_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  cv_bridge
  dynamic_reconfigure
  gazebo_plugins
  gazebo_ros
    gazebo_msgs 
  image_transport
  pcl_ros
  polled_camera
  roscpp
  rospy
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)


include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)


link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    vision_reconfigure 
      gazebo_ros_utils 
    gazebo_ros_utils 
  gazebo_ros_camera_utils 
  gazebo_ros_camera 
  gazebo_ros_multicamera 
  gazebo_ros_depth_camera 
  pcl_ros
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_iggy_vision
#  CATKIN_DEPENDS camera_info_manager cv_bridge cv_bridge dynamic_reconfigure gazebo_plugins gazebo_ros image_transport pcl_ros polled_camera roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


## Declare a C++ library
add_library(${PROJECT_NAME} src/gazebo_iggy_vision.cpp src/whiteline_filter.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(gazebo_iggy_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(gazebo_iggy_vision_node src/gazebo_iggy_vision_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(gazebo_iggy_vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} DepthCameraPlugin ${catkin_LIBRARIES})
