/*
 * DwaMsgs.cpp
 *
 *  Created on: Dec 5, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "dwa_planner_msgs/DwaMsgs.h"

void DwaMsgs::toPathMsg(const Eigen::Vector2d& velocity, double time_horizon, visualization_msgs::Marker& msg)
{
  msg.header.frame_id = "base_link";
  msg.header.stamp = ros::Time::now();
  msg.ns = "dwa";
  msg.id = 0;
  msg.lifetime = ros::Duration();
  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;

  msg.scale.x = 0.01;

  // Default: green color
  msg.color.r = 0.0;
  msg.color.g = 1.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;

  TrajectorySimulator trajectory_sim(velocity, time_horizon);
  for (const auto& robot_position : trajectory_sim.getTrajectory())
  {
    geometry_msgs::Point p;
    p.x = robot_position.x();
    p.y = robot_position.y();
    p.z = 0.1;
    msg.points.push_back(p);
  }
}

void DwaMsgs::toPathMsg(const std::vector<Eigen::Vector2d>& trajectory, visualization_msgs::Marker& msg)
{
  msg.header.frame_id = "base_link";
  msg.header.stamp = ros::Time::now();
  msg.ns = "dwa";
  msg.id = 0;
  msg.lifetime = ros::Duration();
  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;

  msg.scale.x = 0.01;

  // Default: green color
  msg.color.r = 0.0;
  msg.color.g = 1.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;

  for (const auto& robot_position : trajectory)
  {
    geometry_msgs::Point p;
    p.x = robot_position.x();
    p.y = robot_position.y();
    msg.points.push_back(p);
  }
}

void DwaMsgs::toPathMsg(const std::vector<Eigen::Vector2d>& velocity_candidates, double time_horizon,
                        visualization_msgs::MarkerArray& msg)
{
  int32_t path_candidate_id = 0;
  for (const auto& velocity : velocity_candidates)
  {
    visualization_msgs::Marker path_candidate;
    toPathMsg(velocity, time_horizon, path_candidate);

    path_candidate.id = path_candidate_id;
    path_candidate.scale.x = 0.005;

    msg.markers.push_back(path_candidate);
    ++path_candidate_id;
  }
}

void DwaMsgs::toVelocityMsg(const Eigen::Vector2d& velocity, geometry_msgs::Twist& msg)
{
  msg.linear.x = velocity.x();
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = velocity.y();
}

void DwaMsgs::toCircleMsg(const Eigen::Vector2d& center, double radius, visualization_msgs::Marker& msg)
{
  msg.header.frame_id = "base_link";
  msg.header.stamp = ros::Time::now();
  msg.ns = "dwa";
  msg.id = 0;
  msg.lifetime = ros::Duration();
  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;

  msg.scale.x = 0.01;

  // Default: green color
  msg.color.r = 0.0;
  msg.color.g = 1.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;

  const int num_points = 100;
  for (int i = 0; i < num_points; ++i)
  {
    geometry_msgs::Point p;
    p.x = center.x() + radius * cos(2 * M_PI * i / num_points);
    p.y = center.y() + radius * sin(2 * M_PI * i / num_points);
    p.z = 0.1;
    msg.points.push_back(p);
  }
}