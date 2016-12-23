#!/bin/bash		
pwd		
rosdep install -i -r -y --from-paths . --ignore-src --rosdistro indigo

cd ..		
catkin_make --pkg cirkit_waypoint_manager_msgs
source devel/setup.bash		
cd src
