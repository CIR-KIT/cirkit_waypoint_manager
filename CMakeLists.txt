cmake_minimum_required(VERSION 2.8.3)
project(ros_waypoint_generator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  cmake_modules
  interactive_markers
  visualization_msgs
)

find_package(Boost 1.4 COMPONENTS program_options REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp interactive_markers visualization_msgs tf 
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ros_waypoint_generator src/ros_waypoint_generator.cpp)
target_link_libraries(ros_waypoint_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)
