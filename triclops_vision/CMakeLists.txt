cmake_minimum_required(VERSION 2.8.3)
project(triclops_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  image_transport
  cv_bridge
  sensor_msgs
)

# Specify package build info export
catkin_package(
  INCLUDE_DIRS include # This indicates that the folder "include" within the package folder is where exported headers go.
#  LIBRARIES triclops_vision
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


# set(HEADER_FILES   "include/triclops_vision/triclops_camera.h"
#                    "include/triclops_vision/triclops_opencv.h"
#                    "include/triclops_vision/typedefs.h"
#                    "include/triclops_vision/common.h"
#                    "include/triclops_vision/vision_3d.h"
#                    "include/triclops_vision/line_filter.h"
#                    "include/triclops_vision/images_into_triclops.h")

FILE(GLOB_RECURSE HEADER_FILES "include/triclops_vision/*.h")
#message("DDDDDDDDDDDDDDDDDDDDHeaders: " ${HEADER_FILES})
###########
## Build ##
###########
# LIBS    += -ltriclops  -lpnmutils  -lflycapture  -lflycapture2bridge  -lpthread -ldl -lm
find_library(TRICLOPS_LIB triclops)
find_library(PNMUTILS_LIB pnmutils)
find_library(FLYCAPTURE_LIB flycapture)
find_library(FLYCAPTURE2BRIDGE_LIB flycapture2bridge)

find_path(TRICLOPS_INCLUDE_DIR triclops.h HINT "/usr/include/triclops")
find_path(FLYCAPTURE_INCLUDE_DIR FlyCapture2.h HINT "/usr/include/flycapture")

#MESSAGE("=-=-=- cmake ====: ${TRICLOPS_LIB}")
## Specify additional locations of header files
## Your package locations should be listed before other locations
# The first argument "include" indicates that the include/ directory within the package is also part of the path.
set(PROJECT_HEADER_DIR ${PWD}"/include")
include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${TRICLOPS_INCLUDE_DIR}
  ${FLYCAPTURE_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

#message(">>>>>>>>> Inlcude directories:  " ${catkin_INCLUDE_DIRS})
#>>>>>>>>> Inlcude directories:  /opt/ros/indigo/include/usr/include/usr/include/eigen3/usr/include/pcl-1.7/usr/include/ni/usr/include/vtk-5.8/usr/include/opencv

## Declare a cpp library
# add_library(triclops_vision
#   src/${PROJECT_NAME}/triclops_vision.cpp
# )

## Declare a cpp executable
add_executable(triclops_camera_driver src/test_triclops_camera.cpp src/vision_3d.cpp src/triclops_opencv.cpp src/line_filter.cpp ${HEADER_FILES})

add_executable(triclops_image_process src/images_into_triclops.cpp ${HEADER_FILESs})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(triclops_vision_node triclops_vision_generate_messages_cpp)
## Specify libraries to link a library or executable target against

target_link_libraries(triclops_camera_driver
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${TRICLOPS_LIB}
   ${FLYCAPTURE_LIB}
   ${FLYCAPTURE2BRIDGE_LIB}
   ${PNMUTILS_LIB}
 )

target_link_libraries(triclops_image_process
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${TRICLOPS_LIB}
   ${FLYCAPTURE_LIB}
   ${FLYCAPTURE2BRIDGE_LIB}
   ${PNMUTILS_LIB}
 )
