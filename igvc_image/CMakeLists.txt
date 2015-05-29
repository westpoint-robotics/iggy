cmake_minimum_required(VERSION 2.8.3)
project(igvc_image)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy image_transport cv_bridge sensor_msgs pcl_conversions pcl_ros dynamic_reconfigure)
generate_dynamic_reconfigure_options(
 line_filter/cfg/LineFilter.cfg
 image_to_pointcloud/cfg/ImageToPointCloud.cfg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
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
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES igvc_image
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories( include
  ${catkin_INCLUDE_DIRS}
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(igvc_image
#   src/${PROJECT_NAME}/igvc_image.cpp
# )

## Declare a cpp executable
# add_executable(igvc_image_node src/igvc_image_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(igvc_image_node igvc_image_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(igvc_image_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############
add_library(LineFilter line_filter/src/line_filter_nodelet.cc)
add_dependencies(LineFilter ${PROJECT_NAME}_gencfg)

add_executable(LineFilter2 line_filter/src/line_filter.cpp)
target_link_libraries(LineFilter2 ${catkin_LIBRARIES})
add_dependencies(LineFilter2 ${catkin_EXPORTED_TARGETS})


add_executable(DMLineFilter line_filter/src/line_filterDML.cpp)
target_link_libraries(DMLineFilter ${catkin_LIBRARIES})
add_dependencies(DMLineFilter ${catkin_EXPORTED_TARGETS})

add_executable(DMLineFilterRight line_filter/src/line_filterDMLRight.cpp)
target_link_libraries(DMLineFilterRight ${catkin_LIBRARIES})
add_dependencies(DMLineFilterRight ${catkin_EXPORTED_TARGETS})

add_executable(DMLineFilterLeft line_filter/src/line_filterDMLLeft.cpp)
target_link_libraries(DMLineFilterLeft ${catkin_LIBRARIES})
add_dependencies(DMLineFilterLeft ${catkin_EXPORTED_TARGETS})


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# add_executable(linefilterTest line_filter/src/linefilterTest.cpp)
# target_link_libraries(linefilterTest ${catkin_LIBRARIES})
# add_dependencies(linefilterTest ${catkin_EXPORTED_TARGETS})
# 
# add_executable(linefilterV2 line_filter/src/line_filterV2.cpp)
# target_link_libraries(linefilterV2 ${catkin_LIBRARIES})
# add_dependencies(linefilterV2 ${catkin_EXPORTED_TARGETS})




## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS igvc_image igvc_image_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


#add_library(image_to_pointcloud3 image_to_pointcloud/src/image_to_pointcloud3.cc)
#add_dependencies(image_to_pointcloud3 ${PROJECT_NAME}_gencfg)

add_executable(image_to_pointcloud image_to_pointcloud/src/image_to_pointcloud2V3.cpp)
target_link_libraries(image_to_pointcloud ${catkin_LIBRARIES})
add_dependencies(image_to_pointcloud ${catkin_EXPORTED_TARGETS})


add_executable(image_transform image_transform/src/image_transform2.cpp)
target_link_libraries(image_transform ${catkin_LIBRARIES})
add_dependencies(image_transform ${catkin_EXPORTED_TARGETS})
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_igvc_image.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
