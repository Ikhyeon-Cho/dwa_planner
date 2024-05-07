/*
 * RobotTrajectory.h
 *
 *  Created on: Nov 30, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <Eigen/Core>
#include <vector>

class Bubble
{
public:
  Bubble();
  Bubble(const Eigen::Vector2d& position, double radius);
  bool isInside(const Eigen::Vector2d& position) const;
  const Eigen::Vector2d& getPosition() const;
  double getRadius() const;

private:
  Eigen::Vector2d position_;
  double radius_;  // -1 means not initialized
};

class RobotTrajectory
{
public:
  RobotTrajectory(double robot_radius, const Eigen::Vector2d& velocity, double time_horizon);
  bool hasCollisionAt(const Eigen::Vector2d& obstacle) const;

private:
  std::vector<Bubble> trajectory_;
};
