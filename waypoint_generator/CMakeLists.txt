cmake_minimum_required(VERSION 2.8.3)
project(ros_waypoint_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  geometry_msgs
  interactive_markers
  message_generation
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)

find_package(Boost 1.4 COMPONENTS program_options REQUIRED)

add_message_files(
  FILES
  Waypoint.msg
  WaypointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp cmake_modules geometry_msgs interactive_markers  message_runtime nav_msgs roscpp std_msgs tf visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Declare a C++ executable
add_executable(ros_waypoint_generator src/ros_waypoint_generator.cpp)
add_dependencies(ros_waypoint_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_waypoint_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

add_executable(ros_waypoint_saver src/ros_waypoint_saver.cpp)
add_dependencies(ros_waypoint_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_waypoint_saver ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

add_executable(ros_waypoint_server src/ros_waypoint_server.cpp)
add_dependencies(ros_waypoint_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_waypoint_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)
