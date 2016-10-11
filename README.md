# ros_waypoint_generator
This package generate waypoints for 2D navigation by using /map and /odom

## How to use

### required
- /amcl_pose

### run

#### generate waypoint

```bash
$ rosrun ros_waypoint_generator ros_waypoint_generator
```
If you have already had waypoints file as point.csv,  
```bash
$ rosrun ros_waypoint_generator ros_waypoint_generator --load path/to/point.csv
```
The waypoint file format is below.
```
x, y, z, qx, qy, qz, qw, is_searching_area
```
#### save waypoint
```bash
$ rosrun ros_waypoint_generator ros_waypoint_saver
```

### modify_waypoint
If the waypoint area is searching area, the you can make last colum `1`.  
If you set it correct, the marker color will be yellow.  
The normal color is green.  

### parameters
- `dist_th` : threshold of distance for adding new waypoint
- `yaw_th` : threshold of yaw angle[rad] for adding new waypoint

## TODO
- [x] waypointを保存する
- [x] waypointを読み込む
