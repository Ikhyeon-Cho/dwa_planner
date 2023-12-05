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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include <isr_ros_utils/core/core.h>
#include <isr_ros_utils/tools/TransformHandler.h>
#include <isr_ros_utils/tools/PointcloudProcessor.h>

#include "dwa_planner/DwaPlanner.h"
#include "dwa_planner/DwaPlannerRosConverter.h"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

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
  // ROS Parameters : Node
  roscpp::Parameter<std::string> goal_topic{ "LocalPlannerNode/SubscribedTopic/goal", "/move_base_simple/goal" };
  roscpp::Parameter<std::string> pointcloud_topic{ "LocalPlannerNode/SubscribedTopic/pointcloud",
                                                   "/traversability_cloud" };
  roscpp::Parameter<std::string> cmdvel_topic{ "LocalPlannerNode/PublishingTopic/cmd_vel", "/cmd_vel" };

  // ROS Parameters : Framd Ids
  roscpp::Parameter<std::string> frameId_robot{ "frameId_robot", "base_link" };
  roscpp::Parameter<std::string> frameId_map{ "frameId_map", "map" };

  // DWA planner
  roscpp::Publisher<geometry_msgs::Twist> cmdvel_publisher{ cmdvel_topic.param() };
  roscpp::Subscriber<geometry_msgs::PoseStamped> goal_subscriber{ goal_topic.param(), &LocalPlanner::localGoalCallback,
                                                                  this };
  roscpp::Subscriber<sensor_msgs::PointCloud2> pointcloud_subscriber{ pointcloud_topic.param(),
                                                                      &LocalPlanner::pointcloudCallback, this };

  // Dwa planner: velocity constraints
  roscpp::Parameter<double> wheelbase_length{ "LocalPlannerNode/DWA/wheelbase_length", 0.6 };
  roscpp::Parameter<double> max_linear_velocity{ "LocalPlannerNode/DWA/max_linear_velocity", 1.0 };
  roscpp::Parameter<double> max_angular_velocity{ "LocalPlannerNode/DWA/max_angular_velocity", 1.0 };
  roscpp::Parameter<bool> has_max_wheel_velocity{ "LocalPlannerNode/DWA/has_max_wheel_velocity", false };
  roscpp::Parameter<double> max_wheel_vel{ "LocalPlannerNode/DWA/max_wheel_velocity", 1.0 };

  // Dwa planner: cost function parameters
  roscpp::Parameter<double> weight_targetHeading{ "LocalPlannerNode/DWA/target_heading", 1.0 };
  roscpp::Parameter<double> weight_clearance{ "LocalPlannerNode/DWA/clearance", 1.0 };
  roscpp::Parameter<double> weight_velocity{ "LocalPlannerNode/DWA/velocity", 1.0 };
  roscpp::Parameter<double> weight_targetDistance{ "LocalPlannerNode/DWA/target_distance", 1.0 };

  roscpp::Parameter<double> control_duration{ "LocalPlannerNode/DWA/control_duration", 0.1 };  // 10Hz for control
  roscpp::Timer control_timer{ control_duration.param(), &LocalPlanner::controlLoop, this };

  // Velocity Window
  roscpp::Publisher<grid_map_msgs::GridMap> velocity_window_publisher{ "velocity_window", 10 };
  roscpp::Timer visualization_timer_velocityWindow{ ros::Duration(0.2), &LocalPlanner::visualizeVelocityWindow, this };

  roscpp::Publisher<sensor_msgs::PointCloud2> local_cloud_publisher{ "local_cloud" };

  roscpp::Publisher<visualization_msgs::Marker> path_publisher{ "path_best" };
  roscpp::Publisher<visualization_msgs::MarkerArray> path_candidate_publisher{ "path_candidate" };

private:
  DwaPlanner dwa_planner_{ Eigen::Vector2d(max_linear_velocity.param(), max_angular_velocity.param()) };

  TransformHandler transform_handler_;
  PointcloudProcessor<pcl::PointXYZI> pc_processor_;

  geometry_msgs::PoseStamped goal_;
  geometry_msgs::Twist cmd_vel_;
};

}  // namespace ros

#endif