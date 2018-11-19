cmake_minimum_required(VERSION 2.8.3)
project(cirkit_waypoint_navigator)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  jsk_recognition_msgs
  laser_geometry
  message_generation
  move_base_msgs
  roslib
  roscpp
  sensor_msgs
  tf
  visualization_msgs
)

find_package(PkgConfig)
pkg_search_module(EIGEN REQUIRED eigen3)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TeleportAbsolute.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    jsk_recognition_msgs
    laser_geometry
    message_runtime
    move_base_msgs
    roscpp
    roslib
    sensor_msgs
    tf
    visualization_msgs
  DEPENDS EIGEN
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

link_directories(${EIGEN_LIBRARY_DIRS})
## Declare a C++ executable
add_executable(cirkit_waypoint_navigator_node src/cirkit_waypoint_navigator.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(cirkit_waypoint_navigator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(cirkit_waypoint_navigator_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS cirkit_waypoint_navigator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
foreach(dir launch waypoints waypoints/ekiden_final/first waypoints/ekiden_final/second)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

##########
## Test ##
##########
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
