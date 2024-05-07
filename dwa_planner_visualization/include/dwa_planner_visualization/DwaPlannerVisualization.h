/*
 * local_planner_visualization.h
 *
 *  Created on: Nov 30, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef DWA_PLANNER_VISUALIZATION_H
#define DWA_PLANNER_VISUALIZATION_H

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

class DwaPlannerVisualization
{
public:
  DwaPlannerVisualization();

  void velocityWindowCallback(const grid_map_msgs::GridMapConstPtr& msg);

  void visualize(const ros::TimerEvent& event);

public:
  ros::NodeHandle nh_{ "~" };

  std::string velocity_window_topic_{ nh_.param<std::string>("velocityWindowTopic", "dwa_planner/velocity_window") };
  std::string sim_path_topic_{ nh_.param<std::string>("simPathTopic", "dwa_planner/sim_path") };

  ros::Subscriber velocity_window_subscriber_{ nh_.subscribe(velocity_window_topic_, 1,
                                                             &DwaPlannerVisualization::velocityWindowCallback, this) };

  ros::Publisher sim_path_publisher_{ nh_.advertise<nav_msgs::Path>(sim_path_topic_, 1) };

  ros::Timer vis_timer_{ nh_.createTimer(ros::Duration(0.1), &DwaPlannerVisualization::visualize, this) };
  ros::Publisher path_publisher_{ nh_.advertise<visualization_msgs::Marker>("path", 1) };

private:
  grid_map::GridMap velocity_window_;
};

#endif  // DWA_PLANNER_VISUALIZATION_H