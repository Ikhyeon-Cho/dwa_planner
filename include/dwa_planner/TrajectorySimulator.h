/*
 * TrajectorySimulator.h
 *
 *  Created on: Dec 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef TRAJECTORY_SIMULATPR_H
#define TRAJECTORY_SIMULATPR_H

#include <Eigen/Dense>
#include <vector>

class TrajectorySimulator
{
public:
  TrajectorySimulator(const Eigen::Vector2d& velocity, double time_horizon) : v_(velocity.x()), w_(velocity.y())
  {
    const auto& v = velocity.x();
    const auto& w = velocity.y();
    double dt = 0.1;

    Eigen::Vector3d pose(0, 0, 0);
    for (double t = 0; t <= time_horizon; t += dt)
    {
      double dx = v * dt * std::cos(pose.z());
      double dy = v * dt * std::sin(pose.z());
      double dtheta = w * dt;

      pose.x() += dx;
      pose.y() += dy;
      pose.z() += dtheta;

      trajectory_.push_back(pose.head(2));
    }
  }
  
  bool hasCollisionWith(const Eigen::Vector2d& obstacle, double robot_radius) const
  {
    for (auto iterator = trajectory_.rbegin(); iterator != trajectory_.rend(); ++iterator)
    {
      const auto& robot_position = *iterator;
      if ((robot_position - obstacle).norm() < robot_radius)
        return true;
    }
    return false;
  }

  const std::vector<Eigen::Vector2d>& getTrajectory() const
  {
    return trajectory_;
  }

private:
  double v_{ 0 };  // linear velocity
  double w_{ 0 };  // angular velocity
  std::vector<Eigen::Vector2d> trajectory_;
};

#endif