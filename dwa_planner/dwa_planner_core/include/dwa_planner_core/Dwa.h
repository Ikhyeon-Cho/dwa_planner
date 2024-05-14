/*
 * Dwa.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef DWA_CORE_H
#define DWA_CORE_H

#include "velocity_window/VelocityWindow.h"
#include "trajectory_simulator/TrajectorySimulator.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <queue>

class Dwa
{
public:
  /// @brief Constructor of the DWA planner
  /// @param max_v Maximum linear velocity [m/s]
  /// @param max_w Maximum angular velocity [rad/s]
  /// @param velocity_resolution Resolution of the velocity search space [m/s]
  Dwa(double max_v, double max_w, double velocity_resolution, bool enable_backward = false);

  /// @brief Apply Kinematic constraints to velocity search space. We assume two wheel differential drive robot.
  /// @param v_max [rad/s] Maximum wheel velocity
  /// @param wheelbase [m] Wheel base length
  void setKinematicConstraints(double v_max, double wheelbase);

  void setReferenceVelocity(double v_ref);

  /// @brief Setter of the robot size for collision check. Assumed circular robot shape.
  /// @param robot_radius The radius of the robot
  void setRobotSize(double robot_radius);

  /// @brief Setter of the time horizon for collision check.
  /// @param time_horizon Time horizon for collision check [sec]
  void setTimeHorizon(double time_horizon);

  /// @brief Setter of the goal position
  /// @param x x position of the (local) goal
  /// @param y y position of the (local) goal
  void setGoal(double x, double y);

  /// @brief Setter of the target heading cost function weight
  /// @param w_heading The weight of the target heading cost function
  void setTargetHeadingWeight(double w_heading);

  /// @brief Setter of the clearance cost function weight
  /// @param w_clearance The weight of the clearance cost function
  void setClearanceWeight(double w_clearance);

  /// @brief Setter of the velocity cost function weight
  /// @param w_velocity The weight of the velocity cost function
  void setVelocityWeight(double w_velocity);

  /// @brief
  /// @param obstacle
  void updateVelocityObstacles(const pcl::PointCloud<pcl::PointWithRange>::Ptr& obstacle);

  /// @brief Update `TargetHeading`, `Velocity`, `Clearance` window.
  void updateWindow();

  /// @brief Update `TotalCost` and find the best plan from the DWA planner.
  Eigen::Vector2d getBestPlan();

  /// @brief Get the best plan at the specific cost layer
  /// @param layer The layer name of the cost function
  Eigen::Vector2d getBestPlanAt(const std::string& layer) const;

  grid_map::Index getBestPlanVelocityIndex() const;

  bool hasNoPlan() const;

  /// @brief The state of the velocity window.
  /// @return True if the velocity window is pruned by potential collision, otherwise false.
  bool hasPotentialCollision() const;

  bool isNearGoal() const;

  bool hasBackwardGoal() const;

  const VelocityWindow& getVelocityWindow() const;

private:
  void recoverPrunedVelocity();

  void updateTargetHeading();

  void updateClearance();

  void updateVelocity();

  void updateCostFunction();

  // Custom cost functions
  // void updateTargetDistance();

  VelocityWindow velocity_window_;
  Eigen::Vector2d goal_position_{ 0, 0 };
  float max_velocity_{ 1.0 };        // [m/s]
  float reference_velocity_{ 0.6 };  // [m/s]

  // Cost function variables
  double weight_targetHeading_{ 1.0 };
  double weight_clearance_{ 1.0 };
  double weight_velocity_{ 1.0 };
  double time_horizon_{ 3.0 };

  // Collision check variables
  double robot_radius_{ 1.0 };
  pcl::PointCloud<pcl::PointWithRange>::Ptr obstacle_points_{
    boost::make_shared<pcl::PointCloud<pcl::PointWithRange>>()
  };
  std::vector<grid_map::Index> velocity_obstacle_indices_;
};

#endif  // DWA_CORE_H