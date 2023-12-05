/*
 * VelocityWindow.cpp
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "dwa_planner/VelocityWindow.h"

VelocityWindow::VelocityWindow(const Eigen::Vector2d& max_velocity, int n_grid)
  : grid_map::GridMap({ "v", "w", "is_valid" })
{
  setFrameId("base_link");
  setBasicLayers({ "v", "w", "is_valid" });

  auto resolution = (max_velocity.x()) / (n_grid);
  auto length = grid_map::Length(max_velocity.x(), 2 * max_velocity.y());
  setGeometry(length, resolution, grid_map::Position(0.5 * max_velocity.x(), 0));

  initializeVelocitySpace();
}

VelocityWindow::VelocityWindow() : VelocityWindow(Eigen::Vector2d(1.0, 1.0), 100)
{
}

void VelocityWindow::initializeVelocitySpace()
{
  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    Eigen::Vector2d velocity = getVelocityAt(*iterator);
    at("v", *iterator) = velocity.x();
    at("w", *iterator) = velocity.y();
  }

  get("is_valid").setConstant(VALID);
}

Eigen::Vector2d VelocityWindow::getVelocityAt(const grid_map::Index& index) const
{
  grid_map::Position position;
  getPosition(index, position);
  return position;
}

void VelocityWindow::makeValidAt(const grid_map::Index& index)
{
  at("is_valid", index) = VALID;
}

void VelocityWindow::makeInvalidAt(const grid_map::Index& index)
{
  at("is_valid", index) = INVALID;
}

bool VelocityWindow::isInvalidAt(const grid_map::Index& index) const
{
  return (at("is_valid", index) - INVALID) < 1e-3;
}

grid_map::SubmapIterator VelocityWindow::getSquareIterator(const grid_map::Index& query_index, int search_length) const
{
  grid_map::Index submap_start_index = query_index - grid_map::Index(search_length, search_length);
  grid_map::Index submap_buffer_size = grid_map::Index(2 * search_length + 1, 2 * search_length + 1);
  return grid_map::SubmapIterator(*this, submap_start_index, submap_buffer_size);
}