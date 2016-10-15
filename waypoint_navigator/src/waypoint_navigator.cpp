/*-------------------------------------------------
参考プログラム
read_csv.cpp : https://gist.github.com/yoneken/5765597#file-read_csv-cpp

-------------------------------------------------- */


#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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

#define NAVIGATING		0
#define NEARGOAL		1
#define INITNAVI		2


double calculateDistance(double target_x, double target_y , double now_x, double now_y) 
{
  return  sqrt(pow((target_x - now_x), 2.0) + pow((target_y - now_y), 2.0));
}


class WaypointNavigator
{
public:
  WaypointNavigator() : ac_("move_base", true), rate_(100)
  {
    robot_behavior_state_ = RobotBehaviors::State::INIT_NAV;
    stasis = INITNAVI;
    target_num = 0;
    std::string filename;

    ros::NodeHandle n("~");
    n.param<std::string>("waypointsfile", filename, 
                         ros::package::getPath("waypoint_navigator") 
                         + "/waypoints/default.csv");
    n.param("start_target_num", target_num, 0);
    ROS_INFO("[Start target num] : %d", target_num);
    ROS_INFO("[Waypoints file name] : %s", filename.c_str());

    ROS_INFO("Reading Waypoints.");
    readWaypoint(filename.c_str());
   
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
  }

  void sendNewGoal()
  {
    move_base_msgs::MoveBaseGoal goal;
    goal = goals[target_num];
    // Need boost::bind to pass in the 'this' pointer
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal,
                boost::bind(&MyMoveBaseClient::doneCb, this, _1),
                boost::bind(&MyMoveBaseClient::activeCb, this),
                boost::bind(&MyMoveBaseClient::feedbackCb, this, _1));
    stasis = NAVIGATING;
  }

  void doneCb(const actionlib::SimpleClientGoalState& state)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    stasis = INITNAVI;
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Goal just went active");
  }

  // Called every time feedback is received for the goal.
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  {
    //ROS_INFO("[X]: %f [Y]: %f", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y);
    double now_x = feedback->base_position.pose.position.x;
    double now_y = feedback->base_position.pose.position.y;
    double tar_x = goals[target_num-1].target_pose.pose.position.x;
    double tar_y = goals[target_num-1].target_pose.pose.position.y;
    double dist = calculateDistance(tar_x, tar_y, now_x, now_y);
    // ROS_INFO("[num ]: %d", target_num);
    // ROS_INFO("[tarX]: %lf [tarY]: %lf", tar_x, tar_y);
    // ROS_INFO("[nowX]: %lf [nowY]: %lf", now_x, now_y);
    if(dist <= 2.0){
      stasis = NEARGOAL;
      ROS_INFO("Reached WayPT[%d]!!! [Dist] : %lf",target_num, dist);
    }else{
      ROS_INFO("Search WayPT[%d] [Dist]: %lf",target_num, dist);
    }
  }

  int getStasis(){
    return stasis;
  }

  void cancelGoal(){
    ROS_INFO("cancelGoal() is called !!");
    ac.cancelGoal();
  }

  int readWaypoint(std::string filename)
  {
    const int rows_num = 8; // x, y, z, Qx,Qy,Qz,Qw, searching_area
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
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position.x    = data[0];
        goal.target_pose.pose.position.y    = data[1];
        goal.target_pose.pose.position.z    = data[2];
        goal.target_pose.pose.orientation.x = data[3];
        goal.target_pose.pose.orientation.y = data[4];
        goal.target_pose.pose.orientation.z = data[5];
        goal.target_pose.pose.orientation.w = data[6];
        goals.push_back(goal);
        search_area_.push_back((int)data[7]);
        ROS_INFO("[%d]--> [X]: %f [Y]: %f", (int)goals.size(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
      }
    }
    finish_num = (int)goals.size();
    return 0;
  }

  void detectTargetObjectCallback()
  {
  }
  
  void run()
  {
    
    int ret = 0;
    robot_behavior_state_ = RobotBehaviors::State::INIT_NAV;
    while(ros::ok()){
      next_waypoint = this->getNextWaypoint();
      if (next_waypoint.is_search_area) { // 次のwaypointが探索エリアがどうか判定
        if(target_objects_.size() > 0){ // 探索対象が見つかっているか
          for (i = 0; i < target_objects_.size(); ++i) {
            if (! this->isAlreadyApproached(target_objects_[i])) { // 探索対象にまだアプローチしていなかったら
              this->setNextGoal(target_objects_[i], dist_thres_to_apploach_); // 探索対象を次のゴールに設定して
              this->addAlreadyApproached(target_objects_[i]); // 探索対象の位置をアプローチ済みに追加
              robot_behavior_state_ = RobotBehaviors::State::DETECT_TARGET_NAV;
              break;
            }
          }
        }else // 探索エリアだが探索対象がいない
        {
          this->setNextGoal(next_waypoint);
          robot_behavior_state_ = RobotBehaviors::State::WAYPOINT_NAV;
        }
      }else // 探索エリアではない
      {
        this->setNextGoal(next_waypoint);
        robot_behavior_state_ = RobotBehaviors::State::WAYPOINT_NAV;
      }
      while(1)
      {
        robot_current_position = this->getRobotCurrentPosition(); // 現在のロボットの座標
        next_goal_position = this->getNextGoalPosition(); // 現在目指している座標
        distance_to_goal = this->calculateDistance(robot_current_position,
                                                   next_goal_position);
        // ここでスタック判定してもいいかもしれない。
        // 一定時間進んでない、もしくは一定回数後も移動距離が変化していないなら
        // ゴールをキャンセルして次のwaypointに進むとか？
        if(distance_to_goal < this->getGoalThreshold()) // 目標座標までの距離がしきい値になれば
        {
          robot_behavior_state_ = RobotBehaviors::State::REACHED_GOAL;
          break;
        }
        rate_.sleep();
        ros::spinOnce();
      }
      if (robot_behavior_state_ == RobotBehaviors::State::REACHED_GOAL) { //waypointに到達してて
        if (this->isFinalGoal()) { // そのwaypointが最後だったら
          this->cancelGoal(); // ゴールをキャンセルして終了
          return;
        }
      }
      rate.sleep();
      ros::spinOnce();
    }// while(ros::ok())
  }
  
private:
  MoveBaseClient ac_;
  RobotBehaviors::State robot_behavior_state_;
  int target_num_;
  int finish_num_;
  ros::Rate rate_;
  std::vector<move_base_msgs::MoveBaseGoal> goals_;
  std::vector<int> search_area_;
  ros::NodeHandle nh_;
  // ros::ServiceClient client;
  // sound::sound_service srv;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_navigator");
  
  
  WaypointNavigator waypoint_navigator;
 
  waypoint_navigator.run();
 
  return 0;
}
