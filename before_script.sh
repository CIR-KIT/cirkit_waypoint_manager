#!/bin/bash		
pwd		
rosdep install -i -r -y --from-paths . --ignore-src --rosdistro indigo

cd ..		
catkin_make install
source devel/setup.bash		
cd src
