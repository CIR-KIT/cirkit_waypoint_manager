#!/bin/bash		
pwd		
rosdep install -i -r -y --from-paths . --ignore-src --rosdistro indigo

cd ..		
catkin_make
source devel/setup.bash		
catkin_make
source devel/setup.bash		
rm -rf build/ 
cd src
