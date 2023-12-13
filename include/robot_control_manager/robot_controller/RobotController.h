/*
 * RobotController.h
 *
 *  Created on: Dec 13, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Eigen/Dense>

class RobotController
{
public:
  RobotController();

private:
  Eigen::Vector2d velocity_{ 0, 0 };
};

#endif  // ROBOT_CONTROLLER_H