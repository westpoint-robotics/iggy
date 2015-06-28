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

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES triclops_vision
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

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
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${TRICLOPS_INCLUDE_DIR}
  ${FLYCAPTURE_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(triclops_vision
#   src/${PROJECT_NAME}/triclops_vision.cpp
# )

## Declare a cpp executable
#add_executable(triclops_stereoto3d src/stereoto3dpoints.cpp)
# add_executable(triclops_pointcloud_node src/bb2_image_pointCloud.cpp src/LineFilter.cpp)
# add_executable(triclops_stereo_node src/triclops_stereo_node.cpp)
# add_executable(line_detector_node src/line_detector.cpp)

add_executable(usma_line_detector_node src/trans_image.cpp src/LineFilter.cpp)
add_executable(triclops_camera_driver src/test_triclops_camera.cpp src/sto3dpoints.cpp src/triclops_opencv.cpp src/LineFilter.cpp)
add_executable(test_opencvtriclops src/test_opencv2Triclops.cpp src/sto3dpoints.cpp src/triclops_opencv.cpp src/LineFilter.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(triclops_vision_node triclops_vision_generate_messages_cpp)
## Specify libraries to link a library or executable target against


# target_link_libraries(triclops_pointcloud_node
#   ${catkin_LIBRARIES}
#   ${TRICLOPS_LIB}
#   ${FLYCAPTURE_LIB}
#   ${FLYCAPTURE2BRIDGE_LIB}
#   ${PNMUTILS_LIB}
# )

target_link_libraries(triclops_camera_driver
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${TRICLOPS_LIB}
   ${FLYCAPTURE_LIB}
   ${FLYCAPTURE2BRIDGE_LIB}
   ${PNMUTILS_LIB}
 )


target_link_libraries(test_opencvtriclops
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${TRICLOPS_LIB}
   ${FLYCAPTURE_LIB}
   ${FLYCAPTURE2BRIDGE_LIB}
   ${PNMUTILS_LIB}
 )

 target_link_libraries(usma_line_detector_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )
