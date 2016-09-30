#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>
#include <boost/program_options.hpp>

bool compareInteractiveMarker(visualization_msgs::InteractiveMarker left,
                              visualization_msgs::InteractiveMarker right)
{
  return std::stoi(left.name) < std::stoi(right.name);
}

class WaypointSaver
{
public:
  WaypointSaver(const std::string& waypoints_file):
    waypoints_file_(waypoints_file), saved_waypoints_(false)
  {
    ros::NodeHandle n;
    ROS_INFO("Waiting for waypoints");
    waypoints_sub_ = n.subscribe("/cube/update_full", 1, &WaypointSaver::pointsCallback, this);
  }

  void pointsCallback(visualization_msgs::InteractiveMarkerInit all_markers)
  {
    ROS_INFO("Received markers : %d", (int)all_markers.markers.size());
    std::ofstream savefile(waypoints_file_.c_str(), std::ios::out);
    std::sort(all_markers.markers.begin(), all_markers.markers.end(), compareInteractiveMarker);
    size_t size = all_markers.markers.size();
    for(unsigned int i = 0; i < size; i++){
      //３次元の位置の指定
      savefile << all_markers.markers[i].pose.position.x << ","
               << all_markers.markers[i].pose.position.y << ","
               << 0 << ","
               << all_markers.markers[i].pose.orientation.x << ","
               << all_markers.markers[i].pose.orientation.y << ","
               << all_markers.markers[i].pose.orientation.z << ","
               << all_markers.markers[i].pose.orientation.w << std::endl;
    }
    saved_waypoints_ = true;
  }
  
  std::string waypoints_file_;
  ros::Subscriber waypoints_sub_;
  bool saved_waypoints_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_saver");
  std::string waypoints_name = "points.csv";

  WaypointSaver saver(waypoints_name);
  while(!saver.saved_waypoints_ && ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
