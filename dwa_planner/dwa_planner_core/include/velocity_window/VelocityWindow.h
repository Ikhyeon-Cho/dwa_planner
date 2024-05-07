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
  static constexpr float IS_VALID = 1;
  static constexpr float IS_INVALID = 0;

public:
  VelocityWindow(double v_max, double w_max, double velocity_resolution, bool enable_backward = false);

  std::tuple<double, double> getVelocityAt(const grid_map::Index& index) const;
  std::tuple<double, double> getVelocityAt(int) const;
  void makeValidAt(const grid_map::Index& index);
  void makeInvalidAt(const grid_map::Index& index);
  bool isInvalidAt(const grid_map::Index& index) const;

  grid_map::SubmapIterator getSquareIteratorAt(const grid_map::Index& query_index, int search_length) const;
};

#endif