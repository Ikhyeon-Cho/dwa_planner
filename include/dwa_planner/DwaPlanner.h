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

class DwaPlanner
{
public:
  DwaPlanner(const Eigen::Vector2d& max_velocity);

  /// @brief
  /// @param
  /// @param
  /// @param
  void setOptimizationParam(double time_horizon, double w_heading, double w_clearance, double w_velocity);

  /// @brief Apply Kinematic constraints to velocity search space. We assume two wheel differential drive robot.
  /// @param max_wheel_velocity [rad/s] Maximum wheel velocity
  /// @param wheelBase_length [m] Wheel base length
  void setVelocityspaceLimit(double max_wheel_velocity, double wheelBase_length);

  void setCollsionCheckParams(double robot_radius, double time_horizon, int safety_margin);

  void setLocalGoal(const Eigen::Vector2d& goal_position);

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
  // void updateTargetDistance();

  VelocityWindow velocity_space_;
  std::vector<grid_map::Index> pruned_velocity_indices_;
  Eigen::Vector2d goal_position_{ 0, 0 };

  // Cost function parameters
  double weight_targetHeading_{ 1.0 };
  double weight_clearance_{ 1.0 };
  double weight_velocity_{ 1.0 };
  double optimization_timeHorizon_{ 3.0 };

  // Collision check parameters
  double collisionCheck_radius_{ 1.0 };
  double collisionCheck_timehorizon_{ 5.0 };
  int collisionVelocity_margin_{ 5 };
};

#endif