/*
 * LocalPlanner.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_LOCAL_PLANNER_H
#define ROS_LOCAL_PLANNER_H

#include <ros/ros.h>

#include <ros_node_utils/Core.h>
#include <ros_transform_utils/TransformHandler.h>
#include <ros_pcl_utils/PointcloudProcessor.h>

#include "dwa_planner/DwaPlanner.h"
#include "dwa_planner/DwaPlannerRosConverter.h"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

namespace ros
{
class LocalPlanner
{
public:
  LocalPlanner();

  void localGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void visualizeVelocityWindow(const ros::TimerEvent& event);

  void controlLoop(const ros::TimerEvent& event);

public:
  // Subscribed Topics
  roscpp::Parameter<std::string> goal_topic{ "~/Subscribed_Topics/goal", "/move_base_simple/goal" };
  roscpp::Parameter<std::string> pointcloud_topic{ "~/Subscribed_Topics/pointcloud", "/traversability_cloud" };

  // Published Topics
  roscpp::Parameter<std::string> velocity_topic{ "~/Published_Topics/velocity", "/cmd_vel" };
  roscpp::Parameter<std::string> velocityWindow_topic{ "~/Published_Topics/velocity_window", "velocity_window" };

  // Parameters
  // -- Frame Ids
  roscpp::Parameter<std::string> base_frameId{ "~/Parameters/base_frameId", "base_link" };
  roscpp::Parameter<std::string> map_frameId{ "~/Parameters/map_frameId", "map" };
  // -- Velocity constraints
  roscpp::Parameter<double> wheelbase_length{ "~/Parameters/wheelbase_length", 0.6 };
  roscpp::Parameter<double> max_linear_velocity{ "~/Parameters/max_linear_velocity", 1.0 };
  roscpp::Parameter<double> max_angular_velocity{ "~/Parameters/max_angular_velocity", 1.0 };
  roscpp::Parameter<bool> has_max_wheel_velocity{ "~/Parameters/has_max_wheel_velocity", false };
  roscpp::Parameter<double> max_wheel_vel{ "~/Parameters/max_wheel_velocity", 1.0 };
  // -- Cost function parameters
  roscpp::Parameter<double> weight_targetHeading{ "~/Parameters/target_heading", 1.0 };
  roscpp::Parameter<double> weight_clearance{ "~/Parameters/clearance", 1.0 };
  roscpp::Parameter<double> weight_velocity{ "~/Parameters/velocity", 1.0 };
  roscpp::Parameter<double> weight_targetDistance{ "~/Parameters/target_distance", 1.0 };
  // -- Duration
  roscpp::Parameter<double> velocity_publish_duration{ "~/Parameters/velocity_publish_duration", 0.1 };
  roscpp::Parameter<double> velocityWindow_publish_duration{ "~/Parameters/velocityWindow_publish_duration", 0.2 };

private:
  DwaPlanner dwa_planner_{ Eigen::Vector2d(max_linear_velocity.param(), max_angular_velocity.param()) };

  TransformHandler transform_handler_;
  PointcloudProcessor<pcl::PointXYZI> pc_processor_;

  geometry_msgs::PoseStamped goal_;
  geometry_msgs::Twist cmd_vel_;

  roscpp::Subscriber<geometry_msgs::PoseStamped> goal_subscriber{ goal_topic.param(), &LocalPlanner::localGoalCallback,
                                                                  this };
  roscpp::Subscriber<sensor_msgs::PointCloud2> pointcloud_subscriber{ pointcloud_topic.param(),
                                                                      &LocalPlanner::pointcloudCallback, this };
  roscpp::Timer velocity_publish_timer{ velocity_publish_duration.param(), &LocalPlanner::controlLoop, this };
  roscpp::Timer window_visualization_timer{ velocityWindow_publish_duration, &LocalPlanner::visualizeVelocityWindow,
                                            this };

  roscpp::Publisher<geometry_msgs::Twist> cmdvel_publisher{ velocity_topic.param() };
  roscpp::Publisher<grid_map_msgs::GridMap> velocity_window_publisher{ velocityWindow_topic.param() };
  roscpp::Publisher<sensor_msgs::PointCloud2> local_cloud_publisher{ "local_cloud" };
  roscpp::Publisher<visualization_msgs::Marker> path_publisher{ "path_best" };
  roscpp::Publisher<visualization_msgs::MarkerArray> path_candidate_publisher{ "path_candidate" };
};

}  // namespace ros

#endif