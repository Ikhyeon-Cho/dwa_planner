/*
 * VelocityWindow.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef DWA_PLANNER_VELOCITY_WINDOW_H
#define DWA_PLANNER_VELOCITY_WINDOW_H

#include <grid_map_core/grid_map_core.hpp>

class VelocityWindow : public grid_map::GridMap
{
  const float VALID = 1;
  const float INVALID = 0;

public:
  VelocityWindow();
  VelocityWindow(const Eigen::Vector2d& max_velocity, int n_grid);
  void initializeVelocitySpace();

  Eigen::Vector2d getVelocityAt(const grid_map::Index& index) const;
  void makeValidAt(const grid_map::Index& index);
  void makeInvalidAt(const grid_map::Index& index);
  bool isInvalidAt(const grid_map::Index& index) const;

  grid_map::SubmapIterator getSquareIteratorAt(const grid_map::Index& query_index, int search_length) const;
};

#endif