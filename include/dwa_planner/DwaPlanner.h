/*
 * DWAPlanner.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include "dwa_planner/VelocityWindow.h"
#include "dwa_planner/TrajectorySimulator.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <queue>

// Find optimal velocity directly in velocity space

class DwaPlanner
{
public:
  DwaPlanner(const Eigen::Vector2d& max_velocity);

  void setWheelbaseLength(double wheelbase_length);
  void setMaxWheelVelocity(double max_wheel_velocity);
  void setCostWeight(double, double, double, double);

  void setLocalGoal(const Eigen::Vector2d& goal_position);

  // Robot dynamics: assume two-wheeled differential drive robot
  void setVelocityspaceLimit(double max_wheel_velocity);

  void doVelocityPruning(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& obstacle);

  void maximizeObjectiveFunction();

  const VelocityWindow& getVelocityWindow() const;

  Eigen::Vector2d findBestVelocity() const;

private:
  void recoverPrunedVelocity();

  void updateTargetHeading();

  void updateClearance();

  void updateVelocity();

  void updateObjectiveFunction();

  // Custom cost functions
  void updateTargetDistance();

  VelocityWindow velocity_space_;
  std::vector<grid_map::Index> pruned_velocity_indices_;
  Eigen::Vector2d goal_position_{ 0, 0 };

  // Robot-related parameters
  double wheelbase_length_{ 0.5 };
  double max_wheel_vel_{ 1.0 };

  // Cost function parameters
  double weight_targetHeading_{ 1.0 };
  double weight_clearance_{ 1.0 };
  double weight_velocity_{ 1.0 };
  double weight_targetDistance_{ 0.0 };

  // Collision check parameters
  double collisionCheck_timehorizon_{ 3.0 };
};

#endif