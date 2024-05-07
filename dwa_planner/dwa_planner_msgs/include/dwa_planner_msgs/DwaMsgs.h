/*
 * DwaMsgs.h
 *
 *  Created on: Dec 5, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef DWA_PLANNER_ROS_CONVERTER_H
#define DWA_PLANNER_ROS_CONVERTER_H

#include "dwa_planner_core/Dwa.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>

class DwaMsgs
{
public:
  static void toPathMsg(const Eigen::Vector2d& velocity, double time_horizon, visualization_msgs::Marker& msg);

  static void toPathMsg(const Eigen::Vector2d& velocity, double time_horizon, visualization_msgs::Marker& msg,
                        const std::string& color);

  static void toPathMsg(const std::vector<Eigen::Vector2d>& trajectory, visualization_msgs::Marker& msg);

  static void toPathMsg(const std::vector<Eigen::Vector2d>& trajectory, visualization_msgs::Marker& msg,
                        bool has_collision);

  static void toPathMsg(const std::vector<Eigen::Vector2d>& velocity_candidates, double time_horizon,
                        visualization_msgs::MarkerArray& msg);

  static void toVelocityMsg(const Eigen::Vector2d& velocity, geometry_msgs::Twist& msg);
};

#endif