#include "dwa_planner_core/Dwa.h"
// #include "execution_timer/ExecutionTimer.h"

Dwa::Dwa(const Eigen::Vector2d& max_velocity) : velocity_space_(max_velocity, 40)
{
  velocity_space_.add("target_heading", 0.0);
  velocity_space_.add("clearance", 0.0);
  velocity_space_.add("velocity", 0.0);
  // velocity_space_.add("target_distance", 0.0);
  velocity_space_.add("objective_function", 0.0);
}

void Dwa::setLocalGoal(const Eigen::Vector2d& goal_position)
{
  goal_position_ = goal_position;
}

void Dwa::setCollsionCheckParams(double robot_radius, double time_horizon, int safety_margin)
{
  collisionCheck_radius_ = robot_radius;
  collisionCheck_timehorizon_ = time_horizon;
  collisionVelocity_margin_ = safety_margin;
}

void Dwa::setCostParam(double time_horizon, double target_heading, double clearance, double velocity)
{
  optimization_timeHorizon_ = time_horizon;
  weight_targetHeading_ = target_heading;
  weight_clearance_ = clearance;
  weight_velocity_ = velocity;
}

void Dwa::setVelocityspaceLimit(double max_wheel_velocity, double wheelBase_length)
{
  for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
  {
    Eigen::Vector2d velocity = velocity_space_.getVelocityAt(*iterator);

    // convert (v,w) to wheel vel (vl, vr) using kinematics
    // Here, we assume two-wheeled, differential-drive robot
    float rightwheel_vel = velocity.x() + (wheelBase_length * velocity.y()) / 2;  // v_r = v + (w * b /2)
    float leftwheel_vel = velocity.x() - (wheelBase_length * velocity.y()) / 2;   // v_l = v - (w * b /2)

    if (std::abs(rightwheel_vel) > max_wheel_velocity || std::abs(leftwheel_vel) > max_wheel_velocity)
      velocity_space_.makeInvalidAt(*iterator);
  }
}

void Dwa::doVelocityPruning(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
{
  // As this function is trying to do velocity pruning from current obstacle,
  // recovering of previous pruning is required
  recoverPrunedVelocity();

  // for every velocity candidates
  for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
  {
    if (velocity_space_.isInvalidAt(*iterator))
      continue;

    // Collision Check at current velocity
    Eigen::Vector2d velocity = velocity_space_.getVelocityAt(*iterator);
    const auto& v = velocity.x();
    const auto& w = velocity.y();

    for (const auto& obstacle : *cloud)
    {
      // when robot rotates
      if (std::abs(w) > 0.15)
      {
        // instant center of rotation: (0, v/w). i_c radius: |v/w|
        auto i_c_position_x = 0.0;
        auto i_c_position_y = v / w;
        auto i_c_radius = std::abs(i_c_position_y);
        Eigen::Vector2d i_c_position(i_c_position_x, i_c_position_y);
        auto dist_from_ic_to_obstacle = (i_c_position - Eigen::Vector2d(obstacle.x, obstacle.y)).norm();

        // 3. First, check whether the obstacle is inside the robot pathway (circular trajectories)
        // Please refer to Figure.19 of https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf
        if (std::abs(dist_from_ic_to_obstacle - i_c_radius) > collisionCheck_radius_)
          continue;  // no collision arises

        // do trajectory simulation and check collision
        double sim_time =
            (obstacle.intensity > collisionCheck_timehorizon_) ? obstacle.intensity : collisionCheck_timehorizon_;
        TrajectorySimulator trajectory_sim(velocity, sim_time);  // intensity holds distance to obstacle
        if (trajectory_sim.hasCollisionWith(Eigen::Vector2d(obstacle.x, obstacle.y), collisionCheck_radius_))
        {
          velocity_space_.makeInvalidAt(*iterator);
          pruned_velocity_indices_.push_back(*iterator);
          break;
        }
      }

      // robot moves straight. rotation is near zero
      if (std::abs(w) < 0.15 && std::abs(obstacle.x) > v * collisionCheck_timehorizon_)  // out of max position (x-dir)
        continue;

      if (std::abs(w) < 0.15 && std::abs(obstacle.y) > 2 * collisionCheck_radius_)  // outside of predicted position
                                                                                    // (y-dir)
        continue;

      if (std::abs(w) < 0.15)
      {
        double sim_time =
            (obstacle.intensity > collisionCheck_timehorizon_) ? obstacle.intensity : collisionCheck_timehorizon_;
        TrajectorySimulator trajectory_sim(velocity, sim_time);
        if (trajectory_sim.hasCollisionWith(Eigen::Vector2d(obstacle.x, obstacle.y), collisionCheck_radius_))
        {
          velocity_space_.makeInvalidAt(*iterator);
          pruned_velocity_indices_.push_back(*iterator);
          break;
        }
      }
    }  // pointcloud loop ends
  }    // velocity loop ends
}

void Dwa::recoverPrunedVelocity()
{
  if (pruned_velocity_indices_.size() == 0)
    return;

  for (const auto& index : pruned_velocity_indices_)
  {
    velocity_space_.makeValidAt(index);
  }
  pruned_velocity_indices_.clear();
}

void Dwa::maximizeObjectiveFunction()
{
  updateTargetHeading();

  updateClearance();

  updateVelocity();

  // updateTargetDistance();

  updateObjectiveFunction();
}

void Dwa::updateTargetHeading()
{
  auto& targetHeading_cost = velocity_space_["target_heading"];
  for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
  {
    const int i = iterator.getLinearIndex();
    if (velocity_space_.isInvalidAt(*iterator))
    {
      targetHeading_cost(i) = 0;
      continue;
    }

    TrajectorySimulator trajectory_sim(velocity_space_.getVelocityAt(*iterator), optimization_timeHorizon_);
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

void Dwa::updateClearance()
{
  auto max_clearance_cost = collisionVelocity_margin_ * std::sqrt(2);
  velocity_space_["clearance"].setConstant(max_clearance_cost);

  // save current invalid indices
  std::vector<grid_map::Index> obstacle_indices;
  for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
  {
    if (velocity_space_.isInvalidAt(*iterator))
      obstacle_indices.push_back(*iterator);
  }

  auto& clearance_cost = velocity_space_["clearance"];
  for (const auto& index : obstacle_indices)
  {
    // expand invalid indices
    auto search_iterator = velocity_space_.getSquareIteratorAt(index, collisionVelocity_margin_);
    for (search_iterator; !search_iterator.isPastEnd(); ++search_iterator)
    {
      const auto& search_index = *search_iterator;
      grid_map::Position search_position;
      if (!velocity_space_.getPosition(search_index, search_position))
        continue;

      const auto& local_index = search_index - index;
      if (local_index.x() < 0)
        continue;

      auto& cost_at_searchindex = clearance_cost(search_index(0), search_index(1));
      if (cost_at_searchindex < max_clearance_cost - 1e-3 && local_index.y() < 0)
        continue;

      Eigen::Vector2d search_vector(local_index.x(), local_index.y());
      cost_at_searchindex = search_vector.norm();
    }
  }

  clearance_cost = clearance_cost.array() * velocity_space_["is_valid"].array();
  clearance_cost = clearance_cost / clearance_cost.maxCoeff();  // normalize to [0, 1]
}

// void Dwa::updateClearance()
// {
//   const auto& validityCheck_matrix = velocity_space_["is_valid"];
//   auto& clearance_matrix = velocity_space_["clearance"];

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

void Dwa::updateVelocity()
{
  auto& velocity_matrix = velocity_space_["velocity"];
  velocity_matrix = velocity_space_["v"].cwiseProduct(velocity_space_["is_valid"]);

  // Scale to [0, 1]
  velocity_matrix = velocity_matrix / velocity_matrix.maxCoeff();
}

// void Dwa::updateTargetDistance()
// {
//   auto& target_distance_matrix = velocity_space_["target_distance"];
//   for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
//   {
//     const int i = iterator.getLinearIndex();
//     if (velocity_space_.isInvalidAt(*iterator))
//     {
//       target_distance_matrix(i) = -1;
//       continue;
//     }

//     TrajectorySimulator trajectory_sim(velocity_space_.getVelocityAt(*iterator), optimization_timeHorizon_);
//     const auto& predicted_trajectory = trajectory_sim.getTrajectory();
//     if (predicted_trajectory.empty())  // if trajectory simulation is not valid
//     {
//       target_distance_matrix(i) = -1;
//       continue;
//     }

//     // Compute predicted distance to target
//     double min_distance_to_target = std::numeric_limits<double>::max();
//     for (const auto& predicted_position : predicted_trajectory)
//     {
//       const auto& curr_distance_to_target = (predicted_position - goal_position_).norm();
//       min_distance_to_target = std::min(min_distance_to_target, curr_distance_to_target);
//     }
//     target_distance_matrix(i) = min_distance_to_target;
//   }

//   auto max_coeff = target_distance_matrix.maxCoeff();
//   // Replace negative values (-1) with the maximum value using Eigen's array-wise operations
//   target_distance_matrix = (target_distance_matrix.array() < 0).select(max_coeff, target_distance_matrix);
//   auto min_coeff = target_distance_matrix.minCoeff();  // after replacing negative values

//   // Scale to [0, 1]
//   auto max_coeff_matrix =
//       Eigen::MatrixXf::Constant(target_distance_matrix.rows(), target_distance_matrix.cols(), max_coeff);
//   target_distance_matrix = (max_coeff_matrix - target_distance_matrix) / (max_coeff - min_coeff);
// }

void Dwa::updateObjectiveFunction()
{
  const auto& targetHeading = velocity_space_["target_heading"];
  const auto& clearance = velocity_space_["clearance"];
  const auto& velocity = velocity_space_["velocity"];

  auto& objectiveFunction_matrix = velocity_space_["objective_function"];

  // When goal is backside of the robot, use only target heading and clearance
  // Expected behavior: Rotation in the current position
  if (goal_position_.x() < 0)
  {
    objectiveFunction_matrix = weight_targetHeading_ * targetHeading + weight_clearance_ * clearance;
  }
  else
  {
    objectiveFunction_matrix =
        weight_targetHeading_ * targetHeading + weight_clearance_ * clearance + weight_velocity_ * velocity;
  }

  // Scale to [0, 1]
  auto min_coeff = objectiveFunction_matrix.minCoeff();
  auto max_coeff = objectiveFunction_matrix.maxCoeff();
  auto min_coeff_matrix =
      Eigen::MatrixXf::Constant(objectiveFunction_matrix.rows(), objectiveFunction_matrix.cols(), min_coeff);
  objectiveFunction_matrix = (objectiveFunction_matrix - min_coeff_matrix) / (max_coeff - min_coeff);
}

Eigen::Vector2d Dwa::findBestVelocity() const
{
  const auto& objectiveFunction_matrix = velocity_space_["objective_function"];

  float max_coeff = 0;
  grid_map::Index index_of_maxCoeff;
  for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
  {
    const auto& i = iterator.getLinearIndex();
    if (objectiveFunction_matrix(i) >= max_coeff)
    {
      max_coeff = objectiveFunction_matrix(i);
      index_of_maxCoeff = *iterator;
    }
  }

  return velocity_space_.getVelocityAt(index_of_maxCoeff);
}

const VelocityWindow& Dwa::getVelocityWindow() const
{
  return velocity_space_;
}