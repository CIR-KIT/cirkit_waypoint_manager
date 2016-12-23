#!/bin/bash
pwd
cd ..
catkin_make install --pkg cirkit_waypoint_manager_msgs
source devel/setup.bash
cd src
