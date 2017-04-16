/*-------------------------------------------------
参考プログラム
read_csv.cpp : https://gist.github.com/yoneken/5765597#file-read_csv-cpp

-------------------------------------------------- */

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <laser_geometry/laser_geometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <cirkit_waypoint_navigator/TeleportAbsolute.h>

#include <boost/shared_array.hpp>
#include <boost/tokenizer.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "ros_colored_msg.h" // FIXME: this header depend ROS, but exclude ros header. Now must be readed after #include"ros/ros.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

namespace RobotBehaviors {
enum State {
    WAYPOINT_NAV,
    DETECT_TARGET_NAV,
    WAYPOINT_REACHED_GOAL,
    DETECT_TARGET_REACHED_GOAL,
    INIT_NAV,
    WAYPOINT_NAV_PLANNING_ABORTED,
    DETECT_TARGET_NAV_PLANNING_ABORTED
};
}

class WayPoint {
public:
  WayPoint();
  WayPoint(move_base_msgs::MoveBaseGoal goal, int area_type, double reach_threshold)
    : goal_(goal), area_type_(area_type), reach_threshold_(reach_threshold)
  {}
  ~WayPoint(){} // FIXME: Don't declare destructor!!
  bool isSearchArea() {
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


class CirkitWaypointNavigator {
public:
  CirkitWaypointNavigator()
  : ac_("move_base", true),
    rate_(10)
  {
    robot_behavior_state_ = RobotBehaviors::INIT_NAV;
    std::string filename;

    ros::NodeHandle n("~");
    n.param<std::string>("waypointsfile",
        filename,
        ros::package::getPath("cirkit_waypoint_navigator") + "/waypoints/garden_waypoints.csv"); // FIXME: Don't find!

    n.param("dist_thres_to_target_object", dist_thres_to_target_object_, 1.8);
    n.param("limit_of_approach_to_target", limit_of_approach_to_target_, 5);
    n.param("start_waypoint", target_waypoint_index_, 0);
    
    ROS_INFO("[Waypoints file name] : %s", filename.c_str());
    detect_target_objects_sub_ = nh_.subscribe("/recognized_result", 1, &CirkitWaypointNavigator::detectTargetObjectCallback, this);
    detect_target_object_monitor_client_ = nh_.serviceClient<cirkit_waypoint_navigator::TeleportAbsolute>("third_robot_monitor_human_pose");
    next_waypoint_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/next_waypoint", 1);
    ROS_INFO("Reading Waypoints.");
    readWaypoint(filename.c_str());
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
  }

  ~CirkitWaypointNavigator() {
    this->cancelGoal();
  }

  void sendNewGoal(geometry_msgs::Pose pose) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose = pose;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac_.sendGoal(goal);
    now_goal_ = goal.target_pose.pose;
  }

  void sendNextWaypointMarker(const geometry_msgs::Pose waypoint,
      int target_object_mode) {
    visualization_msgs::Marker waypoint_marker;
    waypoint_marker.header.frame_id = "map";
    waypoint_marker.header.stamp = ros::Time();
    waypoint_marker.id = 0;
    waypoint_marker.type = visualization_msgs::Marker::ARROW;
    waypoint_marker.action = visualization_msgs::Marker::ADD;
    waypoint_marker.pose = waypoint;
    waypoint_marker.pose.position.z = 0.3;
    waypoint_marker.scale.x = 0.8;
    waypoint_marker.scale.y = 0.5;
    waypoint_marker.scale.z = 0.1;
    waypoint_marker.color.a = 0.7;
    waypoint_marker.color.r = 0.05 + 1.0*(float)target_object_mode;
    waypoint_marker.color.g = 0.80;
    waypoint_marker.color.b = 0.05 + 1.0*(float)target_object_mode;
    next_waypoint_marker_pub_.publish(waypoint_marker);
  }

  void cancelGoal() {
    ROS_INFO("cancelGoal() is called !!");
    ac_.cancelGoal();
  }

  int readWaypoint(std::string filename) {
    const int rows_num = 9; // x, y, z, Qx,Qy,Qz,Qw, area_type, reach_threshold
    boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
    std::ifstream ifs(filename.c_str());
    std::string line;
    while (ifs.good()) {
      getline(ifs, line);
      if (line.empty()) break;
      tokenizer tokens(line, sep);
      std::vector<double> data;
      tokenizer::iterator it = tokens.begin();
      for (; it != tokens.end() ; ++it) {
        std::stringstream ss;
        double d;
        ss << *it;
        ss >> d;
        data.push_back(d);
      }
      if (data.size() != rows_num) {
        ROS_ERROR("Row size is mismatch!!");
        return -1;
      } else {
        move_base_msgs::MoveBaseGoal waypoint;
        waypoint.target_pose.pose.position.x    = data[0];
        waypoint.target_pose.pose.position.y    = data[1];
        waypoint.target_pose.pose.position.z    = data[2];
        waypoint.target_pose.pose.orientation.x = data[3];
        waypoint.target_pose.pose.orientation.y = data[4];
        waypoint.target_pose.pose.orientation.z = data[5];
        waypoint.target_pose.pose.orientation.w = data[6];
        waypoints_.push_back(WayPoint(waypoint, (int)data[7], data[8]/2.0));
      }
    }
    return 0;
  }

  void detectTargetObjectCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &target_objects_ptr) {
    target_objects_ = *target_objects_ptr;
  }

  WayPoint getNextWaypoint() {
    ROS_INFO_STREAM("Next Waypoint : " << target_waypoint_index_);
    WayPoint next_waypoint = waypoints_[target_waypoint_index_];
    target_waypoint_index_++;
    return next_waypoint;
  }

  bool isFinalGoal() {
    if ((target_waypoint_index_) == ((int)waypoints_.size())) {
      return true;
    }else{
      return false;
    }
  }

  bool isAlreadyApproachedToTargetObject(jsk_recognition_msgs::BoundingBox target_object) {
    for (int i = 0; i < approached_target_objects_.boxes.size(); ++i) {
      double dist = calculateDistance(target_object.pose, approached_target_objects_.boxes[i].pose);
      if (dist < 5.0) { // しきい値はパラメータサーバで設定できるようにする
        return true;
      }
    }
    return false;
  }

  double calculateDistance(geometry_msgs::Pose a,geometry_msgs::Pose b) {
    return sqrt(pow((a.position.x - b.position.x), 2.0) + pow((a.position.y - b.position.y), 2.0));
  }

  // 探索対象へのアプローチの場合
  void setNextGoal(jsk_recognition_msgs::BoundingBox target_object, double threshold) {
    reach_threshold_ = threshold;
    // 現在のロボットの位置と探索対象を中心とした円の交点座標のロボットに近い方
    geometry_msgs::Pose approach_pos = this->getTargetObjectApproachPosition(target_object.pose, 1.0);
    approached_target_objects_.boxes.push_back(target_object);//探索済みに追加
    this->sendNextWaypointMarker(approach_pos, 1);
    this->sendNewGoal(approach_pos);
    // move_baseに渡すgoalはgetTargetObjectApproachPosition()で計算した座標を渡すけど、
    // 実際に探索対象に到達したかどうかの計算には探索対象自体の位置を使いたいから、
    // now_goal_を実際の探索対象の位置で上書きする
    now_goal_ = target_object.pose;
  }

  // 通常のwaypointの場合
  void setNextGoal(WayPoint waypoint) {
    reach_threshold_ = waypoint.reach_threshold_;
    this->sendNextWaypointMarker(waypoint.goal_.target_pose.pose, 0); // 現在目指しているwaypointを表示する
    this->sendNewGoal(waypoint.goal_.target_pose.pose);
  }

  double getReachThreshold() {
    return reach_threshold_;
  }

  geometry_msgs::Pose getRobotCurrentPosition() {
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
    //ROS_INFO_STREAM("c)x :" << pose.position.x << ", y :" << pose.position.y);
    return pose;
  }

  geometry_msgs::Pose getNowGoalPosition() {
    //ROS_INFO_STREAM("g)x :" << now_goal_.position.x << ", y :" << now_goal_.position.y);
    return now_goal_;
  }

  void tryBackRecovery() {
    ROS_INFO_STREAM("Start tryBackRecovery()");
    laser_scan_sub_ = nh_.subscribe("scan_multi", 1, &CirkitWaypointNavigator::laserCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    geometry_msgs::Twist msg;
    geometry_msgs::Pose start_recovery_position = this->getRobotCurrentPosition(); // 現在座標
    ros::Time start_recovery_time = ros::Time::now();
    while (ros::ok()) {
      // 1m 下がる
      int obstacle_counter = 0;
      for (size_t i = 0; i < cloud_.points.size(); ++i) {
        if (0.2 < cloud_.points[i].x && cloud_.points[i].x < 0.7) {
          if (0.3 < cloud_.points[i].y && cloud_.points[i].y < 0.5) {
            obstacle_counter++;
          }
        }
      }
      if(obstacle_counter > 5) {
        msg.linear.x = 0;
        msg.angular.z = 0;
        cmd_vel_pub_.publish(msg);
      } else {
        msg.linear.x = -0.5;
        msg.angular.z = 0;
        cmd_vel_pub_.publish(msg);
      }
      geometry_msgs::Pose robot_current_position = this->getRobotCurrentPosition(); // 現在のロボットの座標
      double delta_dist = this->calculateDistance(robot_current_position,
                                                  start_recovery_position); // 現在位置までの距離を計算
      if(delta_dist > 1.0) { // 1[m]後退したら終わり
        ROS_INFO_STREAM("!! Success to back 1[m] !!");
        break;
      }
      ros::Duration delta_time = ros::Time::now() - start_recovery_time;
      if(delta_time.toSec() > 30) { // 30[s]経過したら終わり
        ROS_WARN_STREAM("Fail to back 1[m], but passed 30[s].");
        break;
      }
      ros::spinOnce();
      rate_.sleep();
    }
    laser_scan_sub_.shutdown();
    cmd_vel_pub_.shutdown();
  }

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    projector_.projectLaser(*scan, cloud_);
  }

  void sendApproachedTargetPosition() {
    jsk_recognition_msgs::BoundingBox approached_target_object = approached_target_objects_.boxes.back();
    cirkit_waypoint_navigator::TeleportAbsolute srv_;
    srv_.request.x = approached_target_object.pose.position.x;
    srv_.request.y = approached_target_object.pose.position.y;
    srv_.request.theta = 0;
    if (detect_target_object_monitor_client_.call(srv_)) {
      ROS_INFO("Succeed to send target object position to server.");
    } else {
      ROS_INFO("Failed to send target object position to server.");
    }
  }

  geometry_msgs::Pose getTargetObjectApproachPosition(geometry_msgs::Pose target_position, double tolerance)
  {
    geometry_msgs::Pose answers[2];
    double distances[2];
    geometry_msgs::Pose robot_position = this->getRobotCurrentPosition();
    if ((robot_position.position.x - target_position.position.x)==0.0 ) {
      answers[0].position.x = target_position.position.x;
      answers[0].position.y = target_position.position.y + tolerance;
      answers[1].position.x = target_position.position.x;
      answers[1].position.y = target_position.position.y - tolerance;
    } else {
      double C = (target_position.position.y - robot_position.position.y)
        / (target_position.position.x - robot_position.position.x);
      answers[0].position.x = target_position.position.x + tolerance/sqrt(1+pow(C, 2.0));
      answers[0].position.y = target_position.position.y + (C*tolerance)/sqrt(1+pow(C, 2.0));
      answers[1].position.x = target_position.position.x - tolerance/sqrt(1+pow(C, 2.0));
      answers[1].position.y = target_position.position.y - (C*tolerance)/sqrt(1+pow(C, 2.0));
    }
    for (size_t i = 0; i < 2; ++i) {
      answers[i].orientation = target_position.orientation;
      distances[i] = this->calculateDistance(robot_position, answers[i]);
    }
    if (distances[0] < distances[1]) {
      return answers[0];
    } else {
      return answers[1];
    }
  }

  void run() {
    robot_behavior_state_ = RobotBehaviors::INIT_NAV;
    number_of_approached_to_target_ = 0;
    while (ros::ok()) {
      bool is_set_next_as_target = false;
      WayPoint next_waypoint = this->getNextWaypoint();
      ROS_GREEN_STREAM("Next WayPoint is got");
      if (next_waypoint.isSearchArea()) { // 次のwaypointが探索エリアがどうか判定
        ROS_INFO_STREAM("Now Search area.");
        if(target_objects_.boxes.size() > 0){ // 探索対象が見つかっているか
          ROS_INFO_STREAM("Found target objects : " << target_objects_.boxes.size());
          for (int i = 0; i < target_objects_.boxes.size(); ++i) {
            if (! this->isAlreadyApproachedToTargetObject(target_objects_.boxes[i])) { // 探索対象にまだアプローチしていなかったら
              //今の位置から5[m]以内なら目指す
              geometry_msgs::Pose robot_pose = this->getRobotCurrentPosition();
              geometry_msgs::Pose target_pose(target_objects_.boxes[i].pose);
              double distance_to_target = this->calculateDistance(robot_pose, target_pose);
              if (distance_to_target < 5.0) {
                ROS_INFO_STREAM("Found new target object.");
                this->setNextGoal(target_objects_.boxes[i], dist_thres_to_target_object_); // 探索対象を次のゴールに設定
                ROS_INFO_STREAM("Set new target_objects as goal.");
                robot_behavior_state_ = RobotBehaviors::DETECT_TARGET_NAV;
                is_set_next_as_target = true;
                break;
              } else { // 探索対象が見つかったが遠すぎる
                ROS_INFO_STREAM("Found new target object, but too far.");
              }
            }
          }
          if (! is_set_next_as_target) {
            this->setNextGoal(next_waypoint);
            robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV;
            is_set_next_as_target = false;
            ROS_INFO_STREAM("Valid target object isn't founded, hence next waypoint is set.");
          }
        } else { // 探索エリアだが探索対象がいない
          ROS_INFO("Searching area but there are not target objects.");
          this->setNextGoal(next_waypoint);
          robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV;
        }
      } else { // 探索エリアではない
        ROS_INFO("Go next_waypoint.");
        this->setNextGoal(next_waypoint);
        robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV;
      }
      ros::Time begin_navigation = ros::Time::now(); // 新しいナビゲーションを設定した時間
      ros::Time verbose_start = ros::Time::now();
      double last_distance_to_goal = 0;
      double delta_distance_to_goal = 1.0; // 0.1[m]より大きければよい
      while (ros::ok()) {
        geometry_msgs::Pose robot_current_position = this->getRobotCurrentPosition(); // 現在のロボットの座標
        geometry_msgs::Pose now_goal_position = this->getNowGoalPosition(); // 現在目指している座標
        double distance_to_goal = this->calculateDistance(robot_current_position, now_goal_position); // 現在位置とwaypointまでの距離を計算
        // ここからスタック(Abort)判定。
        delta_distance_to_goal = last_distance_to_goal - distance_to_goal; // どれだけ進んだか
        if (delta_distance_to_goal < 0.1) { // 進んだ距離が0.1[m]より小さくて
          ros::Duration how_long_stay_time = ros::Time::now() - begin_navigation;
          if (how_long_stay_time.toSec() > 10.0 ) { // 90秒間経過していたら
            if (robot_behavior_state_ == RobotBehaviors::WAYPOINT_NAV) {
              robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV_PLANNING_ABORTED; // プランニング失敗とする
              break;
            } else if (robot_behavior_state_ == RobotBehaviors::DETECT_TARGET_NAV) {
              robot_behavior_state_ = RobotBehaviors::DETECT_TARGET_NAV_PLANNING_ABORTED;
              break;
            } else {
              break;
            }
          } else { // 30秒おきに進捗を報告する
            ros::Duration verbose_time = ros::Time::now() - verbose_start;
            if (verbose_time.toSec() > 30.0) {
              ROS_INFO_STREAM("Waiting Abort: passed 30s, Distance to goal: " << distance_to_goal);
              verbose_start = ros::Time::now();
            }
          }
        } else { // 0.1[m]以上進んでいればOK
          last_distance_to_goal = distance_to_goal;
          begin_navigation = ros::Time::now();
        }
        // waypointの更新判定
        if (distance_to_goal < this->getReachThreshold()) { // 目標座標までの距離がしきい値になれば
          ROS_INFO_STREAM("Distance: " << distance_to_goal);
          if (robot_behavior_state_ == RobotBehaviors::WAYPOINT_NAV) {
            robot_behavior_state_ = RobotBehaviors::WAYPOINT_REACHED_GOAL;
            break;
          } else if(robot_behavior_state_ == RobotBehaviors::DETECT_TARGET_NAV) {
            robot_behavior_state_ = RobotBehaviors::DETECT_TARGET_REACHED_GOAL;
            break;
          } else {
            break;
          }
        }
        rate_.sleep();
        ros::spinOnce();
      }
      switch (robot_behavior_state_) {
        case RobotBehaviors::WAYPOINT_REACHED_GOAL: {
          ROS_INFO("WAYPOINT_REACHED_GOAL");
          if (this->isFinalGoal()) { // そのwaypointが最後だったら
            this->cancelGoal(); // ゴールをキャンセルして終了
            return;
          }
          break;
        }
        case RobotBehaviors::DETECT_TARGET_REACHED_GOAL: {
          ROS_INFO("DETECT_TARGET_REACHED_GOAL");
          this->cancelGoal(); // 探索対象を見つけたらその場で停止して
          ros::Duration(5.0).sleep(); // 5秒停止する
          this->sendApproachedTargetPosition(); // サーバに探索対象の位置を送信する
          // waypointを戻したりするべきかどうか
          // アプローチ回数をリセットする
          number_of_approached_to_target_ = 0;
          target_waypoint_index_ -= 1;
          break;
        }
        case RobotBehaviors::WAYPOINT_NAV_PLANNING_ABORTED: {
          ROS_INFO("!! WAYPOINT_NAV_PLANNING_ABORTED !!");
          this->cancelGoal(); // 今のゴールをキャンセルして
          //this->tryBackRecovery(); // 1mくらい戻ってみて
          target_waypoint_index_ -= 1; // waypoint indexを１つ戻す
          break;
        }
        case RobotBehaviors::DETECT_TARGET_NAV_PLANNING_ABORTED: {
          ROS_INFO("!! DETECT_TARGET_PLANNING_ABORTED !!");
          this->cancelGoal(); // 今の探索対象をキャンセルして
          if (number_of_approached_to_target_ > limit_of_approach_to_target_) {
            // もし何度も同じ探索対象にアプローチしても到達出来なかったら
            // 探索済みに追加したままにしてアプローチ回数をリセットする
            number_of_approached_to_target_ = 0;
          }else{
            // アプローチ回数が一定値以下だったら、
            // 最後に突っ込んだ探索済みとした探索対象を削除する
            ROS_RED_STREAM("Faild to approach ... " << number_of_approached_to_target_ << "times");
            approached_target_objects_.boxes.pop_back();
            number_of_approached_to_target_ += 1;
          }
          target_waypoint_index_ -= 1; // waypoint indexを１つ戻す
          break;
        }
        default: {
          ROS_WARN_STREAM("!! UNKNOWN STATE !!");
          break;
        }
      }
      rate_.sleep();
      ros::spinOnce();
    } // while(ros::ok())
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
  int number_of_approached_to_target_; // １つの探索対象について何度アプローチをしたか
  int limit_of_approach_to_target_; // 1つの探索対象について何度までアプローチするか
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber detect_target_objects_sub_;
  sensor_msgs::LaserScan scan_;
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud cloud_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher next_waypoint_marker_pub_;
  ros::ServiceClient detect_target_object_monitor_client_;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "cirkit_waypoint_navigator");

  CirkitWaypointNavigator cirkit_waypoint_navigator;
  cirkit_waypoint_navigator.run();
 
  return 0;
}
