#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

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

using namespace visualization_msgs;


typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";
  
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      {
        ROS_INFO_STREAM( s.str() << ": pose changed"
                         << "\nposition = "
                         << feedback->pose.position.x
                         << ", " << feedback->pose.position.y
                         << ", " << feedback->pose.position.z
                         << "\norientation = "
                         << feedback->pose.orientation.x
                         << ", " << feedback->pose.orientation.y
                         << ", " << feedback->pose.orientation.z
                         << ", " << feedback->pose.orientation.w
                         << "\nframe: " << feedback->header.frame_id
                         << " time: " << feedback->header.stamp.sec << "sec, "
                         << feedback->header.stamp.nsec << " nsec" );
        break;
      }
  }
  server->applyChanges();
}

class WaypointGenerator
{
public:
  WaypointGenerator():
    rate_(100)
  {
    ros::NodeHandle n("~");
    n.param("dist_th", dist_th_, 2.0); // distance threshold [m]
    n.param("yaw_th", yaw_th_, 30.0*3.1415/180.0); // yaw threshold [rad]
    odom_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",
                              1,
                              &WaypointGenerator::addWaypoint, this);
    waypoint_box_count_ = 0;
    server.reset( new interactive_markers::InteractiveMarkerServer("cube") );
  }

  void load(std::string waypoint_file)
  {
    const int rows_num = 7; // x, y, z, Qx, Qy, Qz, Qw
    boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
    std::ifstream ifs(waypoint_file.c_str());
    std::string line;

    while(ifs.good()){
      getline(ifs, line);
      if(line.empty()){ break; }
      tokenizer tokens(line, sep);
      std::vector<double> data;
      tokenizer::iterator it = tokens.begin();
      for(; it != tokens.end() ; ++it){
        std::stringstream ss;
        double d;
        ss << *it;
        ss >> d;
        data.push_back(d);
      }
      if(data.size() != rows_num){
        ROS_ERROR("Row size is mismatch!!");
        return;
      }else{
        geometry_msgs::PoseWithCovariance new_pose;
        new_pose.pose.position.x = data[0];
        new_pose.pose.position.y = data[1];
        new_pose.pose.position.z = data[2];
        new_pose.pose.orientation.x = data[3];
        new_pose.pose.orientation.y = data[4];
        new_pose.pose.orientation.z = data[5];
        new_pose.pose.orientation.w = data[6];
        makeWaypointMarker(new_pose);
      }
    }
  }

  double calculateDistance(geometry_msgs::PoseWithCovariance new_pose)
  {
    return sqrt(pow(new_pose.pose.position.x - last_pose_.pose.position.x, 2)
                + pow(new_pose.pose.position.y - last_pose_.pose.position.y, 2));
  }

  void getRPY(const geometry_msgs::Quaternion &q,
              double &roll,double &pitch,double &yaw){
    tf::Quaternion tfq(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
  }

  double calculateAngle(geometry_msgs::PoseWithCovariance new_pose)
  {
    double yaw, pitch, roll;
    getRPY(new_pose.pose.orientation, roll, pitch, yaw);
    double last_yaw, last_pitch, last_roll;
    getRPY(last_pose_.pose.orientation, last_roll, last_pitch, last_yaw);
    return fabs(yaw - last_yaw);
  }

  InteractiveMarkerControl& makeWaypointMarkerControl(InteractiveMarker &msg)
  {
    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
    msg.controls.push_back(control);
    control.independent_marker_orientation = true;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    msg.controls.push_back(control);

    Marker marker;
    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale*0.5;
    marker.scale.y = msg.scale*0.5;
    marker.scale.z = msg.scale*0.5;
    marker.color.r = 0.05;
    marker.color.g = 0.80;
    marker.color.b = 0.02;
    marker.color.a = 1.0;

    control.markers.push_back(marker);
    control.always_visible = true;
    msg.controls.push_back(control);

    return msg.controls.back();

  }

  void makeWaypointMarker(const geometry_msgs::PoseWithCovariance new_pose)
  {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.pose = new_pose.pose;
    int_marker.scale = 1;

    std::stringstream s;
    s << waypoint_box_count_;
    int_marker.name = s.str();
    int_marker.description = s.str();

    makeWaypointMarkerControl(int_marker);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    server->applyChanges();
    waypoint_box_count_++;
  }

  void addWaypoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
  {
    double diff_dist = calculateDistance(amcl_pose->pose);
    double diff_yaw = calculateAngle(amcl_pose->pose);
    if(diff_dist > dist_th_ || diff_yaw > yaw_th_)
    {
      makeWaypointMarker(amcl_pose->pose);
      last_pose_ = amcl_pose->pose;
    }
  }

  void run()
  {
    ros::spin();
  }
private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Subscriber odom_sub_;

  geometry_msgs::PoseWithCovariance last_pose_;

  double dist_th_;
  double yaw_th_;
  int waypoint_box_count_;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_generator");
  WaypointGenerator generator;

  boost::program_options::options_description desc("Options");
  desc.add_options()
    ("help", "Print help message")
    ("load", boost::program_options::value<std::string>(), "waypoint filename");

  boost::program_options::variables_map vm;
  try {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);
    if( vm.count("help") ){
      std::cout << "This is waypoint generator node" << std::endl;
      std::cerr << desc << std::endl;
      return 0;
    }
    if (vm.count("load")) {
      generator.load(vm["load"].as<std::string>());
    }
  } catch (boost::program_options::error& e) {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return -1;
  }

  generator.run();
  return 0;
}
