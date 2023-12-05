/*
 * local_planner_visualization.h
 *
 *  Created on: Nov 30, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef LOCAL_PLANNER_VISUALIZATION_H
#define LOCAL_PLANNER_VISUALIZATION_H

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <isr_ros_utils/core/core.h>

namespace ros
{
class LocalPlannerVisualization
{
public:
  LocalPlannerVisualization();

  void velocityWindowCallback(const grid_map_msgs::GridMapConstPtr& msg);

  void visualize(const ros::TimerEvent& event);

public:
  // ROS Parameters: Node
  roscpp::Parameter<std::string> velocity_window_topic{ "LocalPlannerVisualization/SubscribedTopic/velocity_window",
                                                        "dwa_planner/velocity_window" };
  roscpp::Parameter<std::string> sim_path_topic{ "LocalPlannerVisualization/PublishingTopic/sim_path",
                                                 "dwa_planner/sim_path" };

  roscpp::Subscriber<grid_map_msgs::GridMap> velocity_window_subscriber{
    velocity_window_topic.param(), &LocalPlannerVisualization::velocityWindowCallback, this
  };
  roscpp::Publisher<nav_msgs::Path> sim_path_publisher{ sim_path_topic.param() };

  // Test
  roscpp::Timer vis_timer{ ros::Duration(0.1), &LocalPlannerVisualization::visualize, this };
  roscpp::Publisher<visualization_msgs::Marker> path_publisher{ "path" };

private:
  grid_map::GridMap velocity_window_;
};
}  // namespace ros

#endif