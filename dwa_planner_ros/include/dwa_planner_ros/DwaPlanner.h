/*
 * DwaPlanner.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <ros/ros.h>

// Utility
#include "ros_utils/TransformHandler.h"
#include "ros_utils/pointcloud.h"
#include "ros_utils/transform.h"

#include "dwa_planner_core/Dwa.h"
#include "dwa_planner_msgs/DwaMsgs.h"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

class DwaPlanner
{
public:
  DwaPlanner();

  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  /// @brief Transform the goal position to the local frame
  /// @param goal The goal pose in the map frame
  /// @param local_goal The goal pose in the local frame
  /// @return True if transform success, otherwise false
  bool getLocalGoal(const geometry_msgs::PoseStamped& goal, geometry_msgs::PoseStamped& local_goal);

  bool alignToGoal(const geometry_msgs::PoseStamped& goal);

  void plan(const ros::TimerEvent& event);

  bool isArrivedAt(const geometry_msgs::PoseStamped& goal);

  bool requireEmergencyStop() const;

  void laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  double getMinDistanceToObstacle(const pcl::PointCloud<pcl::PointWithRange>::Ptr& obstacle_points);
  
  void publish(const ros::TimerEvent& event);

  void visualizeTrajectories(const ros::TimerEvent& event);


private:
  ros::NodeHandle nh_priv_{ "~" };
  ros_utils::TransformHandler tf_;

  // Topics
  std::string goal_topic_{ nh_priv_.param<std::string>("topic/goal", "/move_base_simple/goal") };
  std::string pointcloud_topic_{ nh_priv_.param<std::string>("topic/pointcloud", "/laser") };

  // Frame Ids
  std::string baselink_frame_{ nh_priv_.param<std::string>("frame_id/baselink", "base_link") };
  std::string map_frame_{ nh_priv_.param<std::string>("frame_id/map", "map") };

  // Robot-specific parameters
  std::vector<double> robot_vel_max_{ nh_priv_.param<std::vector<double>>("robot/maxVelocity", { 1.0, 1.0 }) };
  double robot_max_v_{ robot_vel_max_[0] };
  double robot_max_w_{ robot_vel_max_[1] };
  double robot_ref_v_{ nh_priv_.param<double>("robot/referenceVelocity", 0.6) };
  double wheelbase_{ nh_priv_.param<double>("robot/wheelBaseLength", 0.6) };
  bool enable_backward_motion_{ nh_priv_.param<bool>("robot/enableBackwardMotion", false) };

  // Dwa Parameters: Velocity Window
  double velocity_resolution_{ nh_priv_.param<double>("velocityWindow/resolution", 0.02) };

  // Dwa Parameters: collision check
  double safety_margin_{ nh_priv_.param<double>("collision/safetyMargin", 5) };
  double robot_radius_{ nh_priv_.param<double>("collision/robotRadius", 1.0) };
  double max_laser_range_{ nh_priv_.param<double>("collision/maxLaserRange", 10.0) };

  // Dwa Parameters: cost function
  double w_targetheading_{ nh_priv_.param<double>("cost_function/targetHeading", 1.0) };
  double w_clearance_{ nh_priv_.param<double>("cost_function/clearance", 1.0) };
  double w_velocity_{ nh_priv_.param<double>("cost_function/velocity", 1.0) };
  double time_horizon_{ nh_priv_.param<double>("collision/timeHorizon", 3.0) };

  // Dwa Paramters: Status
  double radius_goal_near_{ nh_priv_.param<double>("status/goalNearRadius", 2.0) };
  double radius_goal_reached_{ nh_priv_.param<double>("status/goalReachedRadius", 0.2) };

  // Publish rates
  double velocity_pub_rate_{ nh_priv_.param<double>("publish_rate/cmdVel", 10) };
  double window_pub_rate_{ nh_priv_.param<double>("publish_rate/dynamicWindow", 5) };
  bool use_debug_{ nh_priv_.param<bool>("publish_rate/use_debug", false) };

  // ROS
  ros::Subscriber sub_goal_{ nh_priv_.subscribe(goal_topic_, 10, &DwaPlanner::goalCallback, this) };
  ros::Subscriber sub_pointcloud_{ nh_priv_.subscribe(pointcloud_topic_, 10, &DwaPlanner::laserCallback, this) };

  ros::Publisher pub_velocity_{ nh_priv_.advertise<geometry_msgs::Twist>("/cmd_vel", 10) };
  ros::Publisher pub_path_best_{ nh_priv_.advertise<visualization_msgs::Marker>("path_best", 1) };
  ros::Publisher pub_path_candidates_{ nh_priv_.advertise<visualization_msgs::MarkerArray>("path_candidates", 1) };
  ros::Publisher pub_path_around_best_{ nh_priv_.advertise<visualization_msgs::MarkerArray>("path_around_best", 1) };

  ros::Timer local_plan_timer_{ nh_priv_.createTimer(velocity_pub_rate_, &DwaPlanner::plan, this, false, false) };

  ros::Timer publish_timer_{ nh_priv_.createTimer(velocity_pub_rate_, &DwaPlanner::publish, this, false, false) };
  ros::Timer window_visualization_timer_{ nh_priv_.createTimer(window_pub_rate_, &DwaPlanner::visualizeTrajectories,
                                                               this) };

  // Publishers for debug purpose
  ros::Publisher pub_obstacle_cloud_;
  ros::Publisher pub_velocity_window_;
  ros::Publisher pub_target_heading_plan_;
  ros::Publisher pub_clearance_plan_;
  ros::Publisher pub_velocity_plan_;

private:
  Dwa dwa_planner_;
  geometry_msgs::PoseStamped msg_goal_;

  // obstacle points
  boost::shared_ptr<pcl::PointCloud<pcl::PointWithRange>> obstacle_points_{
    boost::make_shared<pcl::PointCloud<pcl::PointWithRange>>()
  };
  double min_distance_to_obstacle_{ 1e+4 };
};

#endif  // DWA_PLANNER_H