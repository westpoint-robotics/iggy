cmake_minimum_required(VERSION 2.8.3)
project(igvc_stereo)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(ZED 1.1 REQUIRED)

##For Jetson, OpenCV4Tegra is based on OpenCV2.4
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # Jetson TX1
    SET(OCV_VERSION "2.4")
    SET(CUDA_VERSION "8.0")
    set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
elseif(CMAKE_SYSTEM_NAME2 MATCHES "armv7l" ) # Jetson TK1
    SET(OCV_VERSION "2.4")
    SET(CUDA_VERSION "6.5")
    set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
else() # Ubuntu Desktop
    SET(OCV_VERSION "3.1")
    SET(CUDA_VERSION "8.0")
    message("*** On Ubuntu 14, change line 30 of CMakeLists.txt to CUDA 7.5")
endif()


find_package(OpenCV ${OCV_VERSION} COMPONENTS core highgui imgproc REQUIRED)
find_package(CUDA ${CUDA_VERSION} REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  rosconsole
  sensor_msgs
  dynamic_reconfigure
  tf2_ros
  pcl_conversions
  nodelet
  cv_bridge
)


##############################################################################
# DYNAMIC RECONFIGURE OPTIONS

generate_dynamic_reconfigure_options(
  cfg/color_filter.cfg
  #...
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    sensor_msgs
    opencv
    image_transport
    dynamic_reconfigure
    tf2_ros
    pcl_conversions
)


###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
	${ZED_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

###############################################################################

###############################################################################
# EXECUTABLE

add_definitions(-std=c++11)# -m64) #-Wall)

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})

## This is the thing you were editing
## This is what is (probably) refered to in the .xml
add_executable(zed-ros src/main.cpp src/color_filter.cpp)
target_link_libraries(zed-ros ${LINK_LIBRARIES})
add_dependencies(zed-ros ${PROJECT_NAME}_gencfg)

###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

###############################################################################
# INSTALL

install(TARGETS
  zed-ros ## Rename
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(FILES
#  nodelet_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY
#  launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

















