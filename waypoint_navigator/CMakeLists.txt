cmake_minimum_required(VERSION 2.8.3)
project(waypoint_navigator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  tf
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoint_navigator
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########


# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(waypoint_navigator_node src/waypoint_navigator.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(waypoint_navigator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(waypoint_navigator_node
  ${catkin_LIBRARIES}
)

