/*-------------------------------------------------
参考プログラム
read_csv.cpp : https://gist.github.com/yoneken/5765597#file-read_csv-cpp

-------------------------------------------------- */


#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>

#include <ros/package.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

namespace RobotBehaviors
{
enum State
  {
    WAYPOINT_NAV,
    DETECT_TARGET_NAV,
    REACHED_GOAL,
    INIT_NAV,
  };
}

class WayPoint
{
public:
  WayPoint();
  WayPoint(move_base_msgs::MoveBaseGoal goal, int area_type, double reach_threshold)
    : goal_(goal), area_type_(area_type), reach_threshold_(reach_threshold_)
  {
  }
  ~WayPoint(){}
  bool isSearchArea()
  {
    if (area_type_ == 1) {
      return true;
    }else{
      return false;
    }
  }
  move_base_msgs::MoveBaseGoal goal_;
  int area_type_;
  double reach_threshold_;
};


class WaypointNavigator
{
public:
  WaypointNavigator() : ac_("move_base", true), rate_(100)
  {
    target_waypoint_index_ = 0;
    robot_behavior_state_ = RobotBehaviors::INIT_NAV;
    std::string filename;

    ros::NodeHandle n("~");
    n.param<std::string>("waypointsfile", filename,
                         ros::package::getPath("waypoint_navigator")
                         + "/waypoints/default.csv");

    n.param("dist_thres_to_target_object", dist_thres_to_target_object_, 1.5);
    ROS_INFO("[Waypoints file name] : %s", filename.c_str());

    ROS_INFO("Reading Waypoints.");
    readWaypoint(filename.c_str());
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
  }

  void sendNewGoal(geometry_msgs::Pose pose)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose = pose;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac_.sendGoal(goal);
    now_goal_ = goal.target_pose.pose;
  }

  void cancelGoal(){
    ROS_INFO("cancelGoal() is called !!");
    ac_.cancelGoal();
  }

  int readWaypoint(std::string filename)
  {
    const int rows_num = 9; // x, y, z, Qx,Qy,Qz,Qw, area_type, reach_threshold
    boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
    std::ifstream ifs(filename.c_str());
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
        return -1;
      }else{
        move_base_msgs::MoveBaseGoal waypoint;
        waypoint.target_pose.pose.position.x    = data[0];
        waypoint.target_pose.pose.position.y    = data[1];
        waypoint.target_pose.pose.position.z    = data[2];
        waypoint.target_pose.pose.orientation.x = data[3];
        waypoint.target_pose.pose.orientation.y = data[4];
        waypoint.target_pose.pose.orientation.z = data[5];
        waypoint.target_pose.pose.orientation.w = data[6];
        waypoints_.push_back(WayPoint(waypoint, (int)data[7], data[8]));
      }
    }
    return 0;
  }

  void detectTargetObjectCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &target_objects_ptr)
  {
    target_objects_ = *target_objects_ptr;
  }

  WayPoint getNextWaypoint()
  {
    WayPoint next_waypoint = waypoints_[target_waypoint_index_];
    target_waypoint_index_++;
    return next_waypoint;
  }

  bool isFinalGoal()
  {
    if (target_waypoint_index_ == (int)waypoints_.size()) {
      return true;
    }else{
      return false;
    }
  }

  bool isAlreadyApproachedToTargetObject(jsk_recognition_msgs::BoundingBox target_object)
  {
    for (int i = 0; i < approached_target_objects_.boxes.size(); ++i) {
      double dist = calculateDistance(target_object.pose,
                                      approached_target_objects_.boxes[i].pose);
      if (dist < 3.0) { // しきい値はパラメータサーバで設定できるようにする
        return true;
      }
    }
    return false;
  }

  double calculateDistance(geometry_msgs::Pose a,geometry_msgs::Pose b)
  {
     sqrt(pow((a.position.x - b.position.x), 2.0) +
          pow((a.position.y - b.position.y), 2.0));
  }

  // 探索対象へのアプローチの場合
  void setNextGoal(jsk_recognition_msgs::BoundingBox target_object,
                   double threshold)
  {
    reach_threshold_ = threshold;
    // 探索対象へのアプローチの場合はアプローチ時のロボットの姿勢を計算する必要がある
    // まだ出来てない。
    approached_target_objects_.boxes.push_back(target_object);//探索済みに追加
    this->sendNewGoal(target_object.pose);
  }

  // 通常のwaypointの場合
  void setNextGoal(WayPoint waypoint)
  {
    reach_threshold_ = waypoint.reach_threshold_;
    this->sendNewGoal(waypoint.goal_.target_pose.pose);
  }

  double getReachThreshold()
  {
    return reach_threshold_;
  }

  geometry_msgs::Pose getRobotCurrentPosition()
  {
    // tfを使ってロボットの現在位置を取得する
    tf::StampedTransform transform;
    geometry_msgs::Pose pose;
    try {
      listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    
    return pose;
  }

  geometry_msgs::Pose getNowGoalPosition()
  {
    return now_goal_;
  }
  
  void run()
  {
    robot_behavior_state_ = RobotBehaviors::INIT_NAV;
    while(ros::ok()){
      WayPoint next_waypoint = this->getNextWaypoint();
      if (next_waypoint.isSearchArea()) { // 次のwaypointが探索エリアがどうか判定
        if(target_objects_.boxes.size() > 0){ // 探索対象が見つかっているか
          for (int i = 0; i < target_objects_.boxes.size(); ++i) {
            if (! this->isAlreadyApproachedToTargetObject(target_objects_.boxes[i])) { // 探索対象にまだアプローチしていなかったら
              this->setNextGoal(target_objects_.boxes[i], dist_thres_to_target_object_); // 探索対象を次のゴールに設定
              robot_behavior_state_ = RobotBehaviors::DETECT_TARGET_NAV;
              break;
            }
          }
        }else // 探索エリアだが探索対象がいない
        {
          this->setNextGoal(next_waypoint);
          robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV;
        }
      }else // 探索エリアではない
      {
        this->setNextGoal(next_waypoint);
        robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV;
      }
      while(1)
      {
        geometry_msgs::Pose robot_current_position = this->getRobotCurrentPosition(); // 現在のロボットの座標
        geometry_msgs::Pose now_goal_position = this->getNowGoalPosition(); // 現在目指している座標
        double distance_to_goal = this->calculateDistance(robot_current_position,
                                                          now_goal_position);
        // ここでスタック判定してもいいかもしれない。
        // 一定時間進んでない、もしくは一定回数後も移動距離が変化していないなら
        // ゴールをキャンセルして次のwaypointに進むとか？
        if(distance_to_goal < this->getReachThreshold()) // 目標座標までの距離がしきい値になれば
        {
          robot_behavior_state_ = RobotBehaviors::REACHED_GOAL;
          break;
        }
        rate_.sleep();
        ros::spinOnce();
      }
      if (robot_behavior_state_ == RobotBehaviors::REACHED_GOAL) { //waypointか探索対象に到達してて
        if (this->isFinalGoal()) { // そのwaypointが最後だったら
          this->cancelGoal(); // ゴールをキャンセルして終了
          return;
        }
      }
      rate_.sleep();
      ros::spinOnce();
    }// while(ros::ok())
  }

private:
  MoveBaseClient ac_;
  RobotBehaviors::State robot_behavior_state_;
  ros::Rate rate_;
  std::vector<WayPoint> waypoints_;
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  int target_waypoint_index_; // 次に目指すウェイポイントのインデックス
  jsk_recognition_msgs::BoundingBoxArray target_objects_; //探索対象
  jsk_recognition_msgs::BoundingBoxArray approached_target_objects_; //アプローチ済みの探索対象
  double dist_thres_to_target_object_; // 探索対象にどれだけ近づいたらゴールとするか
  double reach_threshold_; // 今セットされてるゴール（waypointもしくは探索対象）へのしきい値
  geometry_msgs::Pose now_goal_; // 現在目指しているゴールの座標

};


int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_navigator");

  WaypointNavigator waypoint_navigator;
  waypoint_navigator.run();
 
  return 0;
}
