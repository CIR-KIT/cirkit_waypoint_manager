cmake_minimum_required(VERSION 2.8.3)
project(cirkit_waypoint_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  cirkit_waypoint_manager_msgs
  geometry_msgs
  interactive_markers
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)

find_package(Boost 1.4 COMPONENTS program_options REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    interactive_markers
    nav_msgs
    roscpp
    std_msgs
    tf
    visualization_msgs 
  DEPENDS Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Declare a C++ executable
add_executable(cirkit_waypoint_generator src/cirkit_waypoint_generator.cpp)
add_dependencies(cirkit_waypoint_generator ${catkin_EXPORTED_TARGETS})
target_link_libraries(cirkit_waypoint_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

add_executable(cirkit_waypoint_saver src/cirkit_waypoint_saver.cpp)
add_dependencies(cirkit_waypoint_saver ${catkin_EXPORTED_TARGETS})
target_link_libraries(cirkit_waypoint_saver ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

add_executable(cirkit_waypoint_server src/cirkit_waypoint_server.cpp)
add_dependencies(cirkit_waypoint_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(cirkit_waypoint_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lboost_program_options)

# Install
install(TARGETS cirkit_waypoint_generator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
install(TARGETS cirkit_waypoint_saver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
install(TARGETS cirkit_waypoint_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
