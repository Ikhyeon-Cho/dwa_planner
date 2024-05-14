/*
 * VelocityWindow.cpp
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "velocity_window/VelocityWindow.h"
#include <iostream>

VelocityWindow::VelocityWindow(double v_max, double w_max, double velocity_resolution, bool enable_backward)
  : grid_map::GridMap({ "v", "w", "valid_space" })
{
  setFrameId("base_link");
  setBasicLayers({ "v", "w", "valid_space" });

  // Backward motion is allowed
  if (enable_backward)
  {
    auto length = grid_map::Length(2 * v_max, 2 * w_max);
    setGeometry(length, velocity_resolution, grid_map::Position(0, 0));
  }
  else  // Only forward motion is allowed
  {
    auto length = grid_map::Length(v_max, 2 * w_max);
    setGeometry(length, velocity_resolution, grid_map::Position(0.5 * v_max, 0));
  }

  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    auto [v, w] = getVelocityAt(*iterator);
    at("v", *iterator) = v;
    at("w", *iterator) = w;
  }

  get("valid_space").setConstant(VelocityWindow::IS_VALID);
}

std::tuple<double, double> VelocityWindow::getVelocityAt(const grid_map::Index& index) const
{
  grid_map::Position position;
  getPosition(index, position);
  return { position.x(), position.y() };
}

std::tuple<double, double> VelocityWindow::getVelocityAt(int i) const
{
  return { get("v")(i), get("w")(i) };
}

void VelocityWindow::makeValidAt(const grid_map::Index& index)
{
  at("valid_space", index) = VelocityWindow::IS_VALID;
}

void VelocityWindow::makeInvalidAt(const grid_map::Index& index)
{
  at("valid_space", index) = IS_INVALID;
}

bool VelocityWindow::isInvalidAt(const grid_map::Index& index) const
{
  return (at("valid_space", index) - IS_INVALID) < 1e-3;
}

grid_map::SubmapIterator VelocityWindow::getSquareIteratorAt(const grid_map::Index& query_index,
                                                             int search_length) const
{
  grid_map::Index submap_start_index = query_index - grid_map::Index(search_length, search_length);
  grid_map::Index submap_buffer_size = grid_map::Index(2 * search_length + 1, 2 * search_length + 1);
  return grid_map::SubmapIterator(*this, submap_start_index, submap_buffer_size);
}

grid_map::Index VelocityWindow::getMaxVelocityIndex() const
{
  grid_map::Index max_index;
  double max_v = -1e+4;
  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    if (isInvalidAt(*iterator))
      continue;

    auto [v, w] = getVelocityAt(*iterator);
    if (v > max_v)
    {
      max_v = v;
      max_index = *iterator;
    }
  }
  return max_index;
}

int VelocityWindow::getMaxVelocityIndexAt(int idx_column) const
{
  // iteration for row elements (linear velocity space)
  for (int i = getSize()(0) - 1; i >= 0; --i)
  {
    grid_map::Index index(i, idx_column);
    if (isInvalidAt(index))
      return std::max(i - 1, 0);
  }

  return 0;
}