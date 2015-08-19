cmake_minimum_required(VERSION 2.8.3)
project(simple_navigation_goals)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)

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
#  LIBRARIES simple_navigation_goals
#  CATKIN_DEPENDS actionlib move_base_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(simple_navigation_goals
#   src/${PROJECT_NAME}/simple_navigation_goals.cpp
# )

## Declare a cpp executable
# add_executable(simple_navigation_goals_node src/simple_navigation_goals_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(simple_navigation_goals_node simple_navigation_goals_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(simple_navigation_goals_node
#   ${catkin_LIBRARIES}
# )

add_executable(simple_navigation_goal src/simple_nav_goals.cpp)
target_link_libraries(simple_navigation_goal ${catkin_LIBRARIES})

