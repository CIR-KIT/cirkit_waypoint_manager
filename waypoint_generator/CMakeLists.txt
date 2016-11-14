cmake_minimum_required(VERSION 2.8.3)
project(waypoint_generator)

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
  CATKIN_DEPENDS roscpp geometry_msgs interactive_markers message_runtime nav_msgs roscpp std_msgs tf visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Declare a C++ executable
add_executable(waypoint_generator src/waypoint_generator.cpp)
add_dependencies(waypoint_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

add_executable(waypoint_saver src/waypoint_saver.cpp)
add_dependencies(waypoint_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_saver ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

add_executable(waypoint_server src/waypoint_server.cpp)
add_dependencies(waypoint_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)
