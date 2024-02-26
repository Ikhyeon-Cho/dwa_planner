#include "dwa_planner_core/RobotTrajectory.h"

Bubble::Bubble(const Eigen::Vector2d& position, double radius) : position_(position), radius_(radius)
{
}

Bubble::Bubble() : Bubble(Eigen::Vector2d(0, 0), -1)
{
}

const Eigen::Vector2d& Bubble::getPosition() const
{
  return position_;
}

double Bubble::getRadius() const
{
  return radius_;
}

bool Bubble::isInside(const Eigen::Vector2d& position) const
{
  auto distance = (position_ - position).norm();
  if (distance > radius_)
    return false;
  else
    return true;
}

RobotTrajectory::RobotTrajectory(double robot_radius, const Eigen::Vector2d& velocity, double time_horizon)
{
  trajectory_.clear();
  const auto& v = velocity.x();
  const auto& w = velocity.y();
  double dt = 0.5;
  Eigen::Vector3d pose(0, 0, 0);

  for (double t = 0; t <= time_horizon; t += dt)
  {
    double dx = v * dt * std::cos(pose.z());
    double dy = v * dt * std::sin(pose.z());
    double dtheta = w * dt;

    pose.x() += dx;
    pose.y() += dy;
    pose.z() += dtheta;

    trajectory_.push_back(Bubble(pose.head(2), robot_radius));
  }
}

bool RobotTrajectory::hasCollisionAt(const Eigen::Vector2d& obstacle) const
{
  for (const auto& robot : trajectory_)
  {
    if (robot.isInside(obstacle))
      return true;
  }
  return false;
}