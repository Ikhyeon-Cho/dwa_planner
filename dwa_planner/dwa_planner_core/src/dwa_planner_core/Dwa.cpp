#include "dwa_planner_core/Dwa.h"

Dwa::Dwa(double max_v, double max_w, double velocity_resolution, bool enable_backward)
  : velocity_window_(max_v, max_w, velocity_resolution, enable_backward)
{
  velocity_window_.add("TargetHeading", 0.0);
  velocity_window_.add("Velocity", 0.0);
  velocity_window_.add("Clearance", 0.0);
  velocity_window_.add("TotalCost", 0.0);

  obstacle_points_->clear();
}

void Dwa::setKinematicConstraints(double v_max, double wheelbase)
{
  for (grid_map::GridMapIterator iterator(velocity_window_); !iterator.isPastEnd(); ++iterator)
  {
    auto [v, w] = velocity_window_.getVelocityAt(*iterator);

    // convert (v,w) to wheel vel (vl, vr) using kinematics
    // Here, we assume two-wheeled, differential-drive robot
    float vl = v - (w * wheelbase) / 2;  // v_l = v - (w * b /2)
    float vr = v + (w * wheelbase) / 2;  // v_r = v + (w * b /2)

    if (std::abs(vr) > v_max || std::abs(vl) > v_max)
      velocity_window_.makeInvalidAt(*iterator);
  }
}

void Dwa::setRobotSize(double robot_radius)
{
  robot_radius_ = robot_radius;
}

void Dwa::setTimeHorizon(double time_horizon)
{
  time_horizon_ = time_horizon;
}

void Dwa::setReferenceVelocity(double v_ref)
{
  reference_velocity_ = v_ref;
}

void Dwa::updateVelocityObstacles(const pcl::PointCloud<pcl::PointWithRange>::Ptr& cloud)
{
  if (cloud->size() == 0)
  {
    std::cout << "cloud is empty in dwa" << std::endl;
    obstacle_points_->clear();
    return;
  }
  // update obstacle info
  obstacle_points_ = cloud;

  // This function is trying to prune velocity space from current obstacles.
  // Recovering of previous pruning is required. -> TODO: can definitely be improved
  recoverPrunedVelocity();

  // for every velocity candidates
  for (grid_map::GridMapIterator iterator(velocity_window_); !iterator.isPastEnd(); ++iterator)
  {
    if (velocity_window_.isInvalidAt(*iterator))
      continue;

    // Collision Check at current velocity
    auto [v, w] = velocity_window_.getVelocityAt(*iterator);

    for (const auto& obstacle : *cloud)
    {
      // when robot rotates
      double rotation_threshold = 0.15;      // rad/s
      if (std::abs(w) > rotation_threshold)  // rotation_threshold is a heuristic value for deciding rotation.
                                             // less than this value is considered as straight movement
      {
        // instant center of rotation: (0, v/w). i_c radius: |v/w|
        auto i_c_position_x = 0.0;
        auto i_c_position_y = v / w;
        auto i_c_radius = std::abs(i_c_position_y);
        Eigen::Vector2d i_c_position(i_c_position_x, i_c_position_y);
        auto dist_from_ic_to_obstacle = (i_c_position - Eigen::Vector2d(obstacle.x, obstacle.y)).norm();

        // 3. First, check whether the obstacle is inside the robot pathway (circular trajectories)
        // Please refer to Figure.19 of https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf
        if (std::abs(dist_from_ic_to_obstacle) > i_c_radius + robot_radius_)
          continue;  // no collision arises

        // do trajectory simulation and check collision
        TrajectorySimulator trajectory_sim(Eigen::Vector2d(v, w), time_horizon_);
        if (trajectory_sim.hasCollisionWith(Eigen::Vector2d(obstacle.x, obstacle.y), robot_radius_))
        {
          velocity_window_.makeInvalidAt(*iterator);
          velocity_obstacle_indices_.push_back(*iterator);
          break;
        }
      }

      // robot moves straight. rotation is near zero
      if (std::abs(w) < rotation_threshold && std::abs(obstacle.x) > v * time_horizon_)  // out of max position (x-dir)
        continue;

      if (std::abs(w) < rotation_threshold && std::abs(obstacle.y) > 2 * robot_radius_)  // outside of
                                                                                         // predicted position
                                                                                         // (y-dir)
        continue;

      if (std::abs(w) < rotation_threshold)
      {
        TrajectorySimulator trajectory_sim(Eigen::Vector2d(v, w), time_horizon_);
        if (trajectory_sim.hasCollisionWith(Eigen::Vector2d(obstacle.x, obstacle.y), robot_radius_))
        {
          velocity_window_.makeInvalidAt(*iterator);
          velocity_obstacle_indices_.push_back(*iterator);
          break;
        }
      }
    }  // pointcloud loop ends
  }    // velocity loop ends
}

void Dwa::recoverPrunedVelocity()
{
  if (velocity_obstacle_indices_.size() == 0)
    return;

  for (const auto& index : velocity_obstacle_indices_)
  {
    velocity_window_.makeValidAt(index);
  }
  velocity_obstacle_indices_.clear();
}

void Dwa::updateWindow()
{
  updateTargetHeading();

  updateVelocity();

  updateClearance();

  updateCostFunction();
}

void Dwa::updateTargetHeading()
{
  auto& targetHeading_cost = velocity_window_["TargetHeading"];
  for (grid_map::GridMapIterator iterator(velocity_window_); !iterator.isPastEnd(); ++iterator)
  {
    const int i = iterator.getLinearIndex();
    if (velocity_window_.isInvalidAt(*iterator))
    {
      targetHeading_cost(i) = 0;
      continue;
    }

    auto [v, w] = velocity_window_.getVelocityAt(*iterator);

    TrajectorySimulator trajectory_sim(Eigen::Vector2d(v, w), time_horizon_);
    const auto& predicted_trajectory = trajectory_sim.getTrajectory();
    if (predicted_trajectory.size() < 2)
    {
      targetHeading_cost(i) = 0;
      continue;
    }

    // Compute heading error
    const auto& predicted_position = predicted_trajectory.back();
    const auto& predicted_position_prev = predicted_trajectory[predicted_trajectory.size() - 2];  // back of back

    Eigen::Vector2d current_heading_vector = (predicted_position - predicted_position_prev).normalized();
    Eigen::Vector2d target_heading_vector = (goal_position_ - predicted_position).normalized();

    double heading_error = std::acos(current_heading_vector.dot(target_heading_vector));

    // Use [0, 1] range cost: 0 goes to cost 1, PI goes to cost 0
    double heading_error_scaled = 1 - heading_error / M_PI;
    targetHeading_cost(i) = heading_error_scaled;
  }
}

void Dwa::updateVelocity()
{
  auto& velocity_matrix = velocity_window_["Velocity"];

  // make velocity_matrix as always positive absolute value
  // y = -(x - v_ref)^2 + v_ref
  velocity_matrix = velocity_window_["v"].cwiseProduct(velocity_window_["valid_space"]).cwiseAbs();

  // const auto& v_ref = reference_velocity_;
  // velocity_matrix = -1 / v_ref * (velocity_window_["v"].array() - v_ref).square() + v_ref;
  // velocity_matrix = velocity_matrix.cwiseProduct(velocity_window_["valid_space"]);

  // No valid velocities
  if (velocity_matrix.maxCoeff() < 1e-3)
    return;

  // Scale to [0, 1]
  velocity_matrix = velocity_matrix / velocity_matrix.maxCoeff();
}

void Dwa::updateClearance()
{
  // calculate the distance from the closest obstacle
  auto& clearance_cost = velocity_window_["Clearance"];

  // Skip if there is no obstacle
  if (obstacle_points_->size() == 0)
  {
    clearance_cost.setOnes();
    return;
  }

  // For every velocity candidates
  for (grid_map::GridMapIterator iterator(velocity_window_); !iterator.isPastEnd(); ++iterator)
  {
    const int i = iterator.getLinearIndex();
    if (velocity_window_.isInvalidAt(*iterator))
    {
      clearance_cost(i) = 0;
      continue;
    }
    auto [v, w] = velocity_window_.getVelocityAt(i);

    // Forward Simulation
    TrajectorySimulator trajectory_sim(Eigen::Vector2d(v, w), time_horizon_);
    const auto& predicted_trajectory = trajectory_sim.getTrajectory();
    if (predicted_trajectory.size() == 0)
    {
      clearance_cost(i) = 0;
      continue;
    }

    // Compute the closest obstacle position from predicted position
    double min_distance = 1e+4;
    auto predicted_position = predicted_trajectory.back();
    for (const auto& obstacle : *obstacle_points_)
    {
      double distance = (predicted_position - Eigen::Vector2d(obstacle.x, obstacle.y)).norm();
      min_distance = std::min(min_distance, distance);
    }

    clearance_cost(i) = min_distance * std::exp(min_distance);
  }  // for loop ends

  // Scale to [0, 1]
  if (clearance_cost.maxCoeff() < 1e-3)
  {
    clearance_cost.setZero();
    return;
  }
  else
  {
    clearance_cost =
        (clearance_cost.array() - clearance_cost.minCoeff()) / (clearance_cost.maxCoeff() - clearance_cost.minCoeff());
  }
}

// void Dwa::updateClearance()
// {
//   auto max_clearance_cost = collisionVelocity_margin_ * std::sqrt(2);
//   velocity_window_["Clearance"].setConstant(max_clearance_cost);

//   // save current invalid indices
//   std::vector<grid_map::Index> obstacle_indices;
//   for (grid_map::GridMapIterator iterator(velocity_window_); !iterator.isPastEnd(); ++iterator)
//   {
//     if (velocity_window_.isInvalidAt(*iterator))
//       obstacle_indices.push_back(*iterator);
//   }

//   auto& clearance_cost = velocity_window_["Clearance"];
//   for (const auto& index : obstacle_indices)
//   {
//     // expand invalid indices
//     auto search_iterator = velocity_window_.getSquareIteratorAt(index, collisionVelocity_margin_);
//     for (search_iterator; !search_iterator.isPastEnd(); ++search_iterator)
//     {
//       const auto& search_index = *search_iterator;
//       grid_map::Position search_position;
//       if (!velocity_window_.getPosition(search_index, search_position))
//         continue;

//       const auto& local_index = search_index - index;
//       if (local_index.x() < 0)
//         continue;

//       auto& cost_at_searchindex = clearance_cost(search_index(0), search_index(1));
//       if (cost_at_searchindex < max_clearance_cost - 1e-3 && local_index.y() < 0)
//         continue;

//       Eigen::Vector2d search_vector(local_index.x(), local_index.y());
//       cost_at_searchindex = search_vector.norm();
//     }
//   }

//   clearance_cost = clearance_cost.array() * velocity_window_["valid_space"].array();
//   clearance_cost = clearance_cost / clearance_cost.maxCoeff();  // normalize to [0, 1]
// }

// void Dwa::updateClearance()
// {
//   const auto& validityCheck_matrix = velocity_space_["valid_space"];
//   auto& clearance_matrix = velocity_space_["Clearance"];

//   auto n_row = validityCheck_matrix.rows();
//   auto n_col = validityCheck_matrix.cols();

//   clearance_matrix = validityCheck_matrix;

//   int add_clearance = 10;

//   // Expand invalid velocity space downwards (decreased linear velocity near invalid)
//   // Shift validity check matrix to downwards
//   Eigen::MatrixXf shifted_matrix(n_row, n_col);
//   shifted_matrix.block(add_clearance, 0, n_row - add_clearance, n_col) =
//       validityCheck_matrix.block(0, 0, n_row - add_clearance, n_col);
//   shifted_matrix.block(0, 0, add_clearance, n_col) = clearance_matrix.block(0, 0, add_clearance, n_col);

//   for (size_t i = 0; i < add_clearance; ++i)
//   {
//     shifted_matrix.row(i) = validityCheck_matrix.row(0);
//   }

//   clearance_matrix = shifted_matrix;
// }

void Dwa::updateCostFunction()
{
  const auto& targetHeading = velocity_window_["TargetHeading"];
  const auto& clearance = velocity_window_["Clearance"];
  const auto& velocity = velocity_window_["Velocity"];
  auto& cost_matrix = velocity_window_["TotalCost"];

  // Remove avoidance behavior when robot is near the goal
  // Clear goal should be guaranteed by global planner
  if (isNearGoal())
  {
    cost_matrix = (2 * weight_targetHeading_ * targetHeading) + (weight_velocity_ * velocity);
  }
  // clearance weight is adaptively adjusted by using closest obstacle distance
  else if (hasPotentialCollision())
  {
    cost_matrix =
        (0.1 * weight_targetHeading_ * targetHeading) + (weight_velocity_ * velocity) + (weight_clearance_ * clearance);
  }
  else
  {
    cost_matrix =
        (weight_targetHeading_ * targetHeading) + (weight_velocity_ * velocity) + (weight_clearance_ * clearance);
  }

  // Scale to [0, 1]
  auto min_coeff = cost_matrix.minCoeff();
  auto max_coeff = cost_matrix.maxCoeff();
  auto min_coeff_matrix = Eigen::MatrixXf::Constant(cost_matrix.rows(), cost_matrix.cols(), min_coeff);
  cost_matrix = (cost_matrix - min_coeff_matrix) / (max_coeff - min_coeff);
}

bool Dwa::hasPotentialCollision() const
{
  // grid map iter
  for (grid_map::GridMapIterator iterator(velocity_window_); !iterator.isPastEnd(); ++iterator)
  {
    if (velocity_window_.isInvalidAt(*iterator))
    {
      std::cout << "has potential collision" << std::endl;
      return true;
    }
  }

  std::cout << "no potential collision" << std::endl;
  return false;

  if (velocity_obstacle_indices_.size() > 0)
  {
    std::cout << "has potential collision" << std::endl;
    return true;
  }
  else
  {
    std::cout << "no potential collision" << std::endl;

    return false;
  }
}

Eigen::Vector2d Dwa::getBestPlan()
{
  updateCostFunction();

  return getBestPlanAt("TotalCost");
}

Eigen::Vector2d Dwa::getBestPlanAt(const std::string& layer) const
{
  const auto& cost_matrix = velocity_window_[layer];

  float max_cost = 0;
  grid_map::Index max_cost_index;

  for (grid_map::GridMapIterator iterator(velocity_window_); !iterator.isPastEnd(); ++iterator)
  {
    const auto& i = iterator.getLinearIndex();
    if (cost_matrix(i) >= max_cost)
    {
      max_cost = cost_matrix(i);
      max_cost_index = *iterator;
    }
  }
  auto [v, w] = velocity_window_.getVelocityAt(max_cost_index);
  return Eigen::Vector2d(v, w);
}

bool Dwa::hasNoPlan() const
{
  // count the number of valid velocities
  int n_valid_velocities = 0;
  for (grid_map::GridMapIterator iterator(velocity_window_); !iterator.isPastEnd(); ++iterator)
  {
    if (!velocity_window_.isInvalidAt(*iterator))
      n_valid_velocities++;
  }

  // if the percent of valid velocities is less than 5%, return true
  if (n_valid_velocities < 0.05 * velocity_window_.getSize().prod())
    return true;

  return false;
}

bool Dwa::isNearGoal() const
{
  return (goal_position_).norm() < 2.0 + robot_radius_;
}

bool Dwa::hasBackwardGoal() const
{
  return goal_position_.x() < 0;
}

void Dwa::setGoal(double x, double y)
{
  goal_position_ = Eigen::Vector2d(x, y);
}

const VelocityWindow& Dwa::getVelocityWindow() const
{
  return velocity_window_;
}

void Dwa::setTargetHeadingWeight(double w_heading)
{
  weight_targetHeading_ = w_heading;
}

void Dwa::setClearanceWeight(double w_clearance)
{
  weight_clearance_ = w_clearance;
}

void Dwa::setVelocityWeight(double w_velocity)
{
  weight_velocity_ = w_velocity;
}

grid_map::Index Dwa::getBestPlanVelocityIndex() const
{
  const auto& cost_matrix = velocity_window_["TotalCost"];

  float max_cost = 0;
  grid_map::Index max_cost_index;

  for (grid_map::GridMapIterator iterator(velocity_window_); !iterator.isPastEnd(); ++iterator)
  {
    const auto& i = iterator.getLinearIndex();
    if (cost_matrix(i) >= max_cost)
    {
      max_cost = cost_matrix(i);
      max_cost_index = *iterator;
    }
  }
  return max_cost_index;
}