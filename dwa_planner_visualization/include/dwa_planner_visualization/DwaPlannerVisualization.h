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

#include <ros_node_utils/Core.h>

class DwaPlannerVisualization
{
public:
  DwaPlannerVisualization();

  void velocityWindowCallback(const grid_map_msgs::GridMapConstPtr& msg);

  void visualize(const ros::TimerEvent& event);

public:
  // ROS Parameters: Node
  roscpp::Parameter<std::string> velocity_window_topic{ "DwaPlannerVisualization/SubscribedTopic/velocity_window",
                                                        "dwa_planner/velocity_window" };
  roscpp::Parameter<std::string> sim_path_topic{ "DwaPlannerVisualization/PublishingTopic/sim_path",
                                                 "dwa_planner/sim_path" };

  roscpp::Subscriber<grid_map_msgs::GridMap> velocity_window_subscriber{
    velocity_window_topic.param(), &DwaPlannerVisualization::velocityWindowCallback, this
  };
  roscpp::Publisher<nav_msgs::Path> sim_path_publisher{ sim_path_topic.param() };

  // Test
  roscpp::Timer vis_timer{ ros::Duration(0.1), &DwaPlannerVisualization::visualize, this };
  roscpp::Publisher<visualization_msgs::Marker> path_publisher{ "path" };

private:
  grid_map::GridMap velocity_window_;
};

#endif  // DWA_PLANNER_VISUALIZATION_H