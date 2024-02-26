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

  void localGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void visualizeVelocityWindow(const ros::TimerEvent& event);

  void DWA(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_{ "~" };
  ros_utils::TransformHandler tf_handler_;

  // Topics
  std::string goal_topic_{ nh_.param<std::string>("goalTopic", "/move_base_simple/goal") };
  std::string pointcloud_topic_{ nh_.param<std::string>("pointCloudTopic", "/obstacle_cloud") };
  std::string velocity_topic_{ nh_.param<std::string>("velocityTopic", "/cmd_vel") };
  std::string velocityWindow_topic_{ nh_.param<std::string>("velocityWindowTopic", "velocity_window") };

  // Frame Ids
  std::string baselink_frame_{ nh_.param<std::string>("baselinkFrame", "base_link") };
  std::string map_frame_{ nh_.param<std::string>("mapFrame", "map") };

  // Robot-specific parameters
  double wheelbase_length_{ nh_.param<double>("wheelBaseLength", 0.6) };
  double max_linear_vel_{ nh_.param<double>("maxLinearVelocity", 1.0) };
  double max_angular_vel_{ nh_.param<double>("maxAngularVelocity", 1.0) };

  // Dwa Parameters: collision check
  double robot_radius_{ nh_.param<double>("robotRadius", 1.0) };
  int safety_margin_{ nh_.param<int>("safetyMargin", 5) };
  double collision_check_timehorizon_{ nh_.param<double>("collisionCheckTimeHorizon", 3.0) };

  // Dwa Parameters: cost function
  double optimization_timehorizon_{ nh_.param<double>("optimizationTimeHorizon", 3.0) };
  double weight_targetheading_{ nh_.param<double>("targetHeading", 1.0) };
  double weight_clearance_{ nh_.param<double>("clearance", 1.0) };
  double weight_velocity_{ nh_.param<double>("velocity", 1.0) };

  // ROS Node Duration
  double velocity_pub_rate_{ nh_.param<double>("velocityPubRate", 10) };
  double window_pub_rate_{ nh_.param<double>("dynamicWindowPubRate", 5) };

  // ROS
  ros::Subscriber sub_goal_{ nh_.subscribe(goal_topic_, 10, &DwaPlanner::localGoalCallback, this) };
  ros::Subscriber sub_pointcloud_{ nh_.subscribe(pointcloud_topic_, 10, &DwaPlanner::pointcloudCallback, this) };

  ros::Publisher pub_velocity_{ nh_.advertise<geometry_msgs::Twist>(velocity_topic_, 10) };
  ros::Publisher pub_velocity_window_{ nh_.advertise<grid_map_msgs::GridMap>(velocityWindow_topic_, 1) };
  ros::Publisher pub_obstacle_cloud_{ nh_.advertise<sensor_msgs::PointCloud2>("local_cloud", 1) };
  ros::Publisher pub_path_best_{ nh_.advertise<visualization_msgs::Marker>("path_best", 1) };
  ros::Publisher pub_path_candidates_{ nh_.advertise<visualization_msgs::MarkerArray>("path_candidates", 1) };

  ros::Timer local_plan_timer_{ nh_.createTimer(velocity_pub_rate_, &DwaPlanner::DWA, this, false, false) };
  ros::Timer window_visualization_timer_{ nh_.createTimer(window_pub_rate_, &DwaPlanner::visualizeVelocityWindow, this) };

private:
  Dwa dwa_planner_{ Eigen::Vector2d{ max_linear_vel_, max_angular_vel_ } };

  geometry_msgs::PoseStamped goal_;
  geometry_msgs::Twist cmd_vel_;
};

#endif  // DWA_PLANNER_H