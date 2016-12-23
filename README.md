# cirkit_waypoint_manager [![Build Status](https://travis-ci.org/CIR-KIT/cirkit_waypoint_manager.svg?branch)](https://travis-ci.org/CIR-KIT/cirkit_waypoint_manager) [![Slack](https://img.shields.io/badge/Slack-CIR--KIT-blue.svg)](http://cir-kit.slack.com/messages/cirkit_waypoint_manager)
Waypoint management package for move_base
Supported ROS version is **Indigo**.

---
## Installation
##### 1. Create **catkinized**  workspace.
##### 2. Clone this repository.
```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/CIR-KIT/cirkit_waypoint_manager.git
```
##### 3. Download required packages by wstool.
```bash
$ cd <catkin_ws>
$ wstool init src
$ wstool merge -t src src/cirkit_waypoint_manager/cirkit_waypoint_manager.rosinstall
$ wstool update -t src
```
##### 4. Download depended packages by rosdep.
```bash
$ cd <catkin_ws>
$ rosdep install -i -r -y --from-paths src --ignore-src
```
##### 5. Build packages, and set the path for the packages.
```bash
$ cd <catkin_ws>
$ catkin_make
$ source devel/setup.bash
```
