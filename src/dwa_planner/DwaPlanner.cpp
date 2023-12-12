#include "dwa_planner/DwaPlanner.h"
#include "execution_timer/ExecutionTimer.h"

DwaPlanner::DwaPlanner(const Eigen::Vector2d& max_velocity) : velocity_space_(max_velocity, 40)
{
  velocity_space_.add("target_heading", 0.0);
  velocity_space_.add("clearance", 0.0);
  velocity_space_.add("velocity", 0.0);
  velocity_space_.add("target_distance", 0.0);
  velocity_space_.add("objective_function", 0.0);
}

void DwaPlanner::setWheelbaseLength(double wheelbase_length)
{
  wheelbase_length_ = wheelbase_length;
}

void DwaPlanner::setMaxWheelVelocity(double max_wheel_velocity)
{
  max_wheel_vel_ = max_wheel_velocity;
}

void DwaPlanner::setLocalGoal(const Eigen::Vector2d& goal_position)
{
  goal_position_ = goal_position;
}

void DwaPlanner::setRobotRadius(double robot_radius)
{
  robot_radius_ = robot_radius;
}

void DwaPlanner::setCostWeight(double target_heading, double clearance, double velocity, double target_distance)
{
  weight_targetHeading_ = target_heading;
  weight_clearance_ = clearance;
  weight_velocity_ = velocity;
  weight_targetDistance_ = target_distance;
}

void DwaPlanner::setVelocityspaceLimit(double max_wheel_velocity)
{
  for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
  {
    Eigen::Vector2d velocity = velocity_space_.getVelocityAt(*iterator);

    // convert (v,w) to wheel vel (vl, vr) using kinematics
    // Here, we assume two-wheeled, differential-drive robot
    float rightwheel_vel = velocity.x() + (wheelbase_length_ * velocity.y()) / 2;  // v_r = v + (w * b /2)
    float leftwheel_vel = velocity.x() - (wheelbase_length_ * velocity.y()) / 2;   // v_l = v - (w * b /2)

    if (std::abs(rightwheel_vel) > max_wheel_velocity || std::abs(leftwheel_vel) > max_wheel_velocity)
      velocity_space_.makeInvalidAt(*iterator);
  }
}

void DwaPlanner::doVelocityPruning(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
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

    // Skip redundant obstacles: thinking about current velocity removes many obstacle candidates
    for (const auto& obstacle : *cloud)
    {
      // 1. robot turns left -> skip right-hand side obstacles
      if (w > 1e-2 && obstacle.y < 0)
        continue;

      // 2. robot turns right -> skip left-hand side obstacles
      if (w < -1e-2 && obstacle.y > 0)
        continue;

      // robot rotates
      if (std::abs(w) > 1e-2)
      {
        // instant center of rotation: (0, v/w). i_c radius: |v/w|
        auto i_c_position_x = 0.0;
        auto i_c_position_y = v / w;
        auto i_c_radius = std::abs(i_c_position_y);
        Eigen::Vector2d i_c_position(i_c_position_x, i_c_position_y);
        auto from_ic_to_obstacle = (i_c_position - Eigen::Vector2d(obstacle.x, obstacle.y)).norm();

        // 3. First, check whether the obstacle is inside the robot pathway (circular trajectories)
        // Please refer to Figure.19 of https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf
        if (std::abs(from_ic_to_obstacle - i_c_radius) > robot_radius_)
          continue;  // no collision arises

        // do trajectory simulation and check collision
        double sim_time =
            (obstacle.intensity > collisionCheck_timehorizon_) ? obstacle.intensity : collisionCheck_timehorizon_;
        TrajectorySimulator trajectory_sim(velocity, sim_time);  // intensity holds distance to obstacle
        if (trajectory_sim.hasCollisionWith(Eigen::Vector2d(obstacle.x, obstacle.y), robot_radius_))
        {
          velocity_space_.makeInvalidAt(*iterator);
          pruned_velocity_indices_.push_back(*iterator);
          break;
        }
      }

      // robot moves straight. rotation is near zero
      if (std::abs(w) < 1e-2 && std::abs(obstacle.x) > v * collisionCheck_timehorizon_)  // outside of predicted
                                                                                         // position (x-dir)
        continue;

      if (std::abs(w) < 1e-2 && std::abs(obstacle.y) > robot_radius_)  // outside of predicted position (y-dir)
        continue;

      if (std::abs(w) < 1e-2)
      {
        double sim_time =
            (obstacle.intensity > collisionCheck_timehorizon_) ? obstacle.intensity : collisionCheck_timehorizon_;
        TrajectorySimulator trajectory_sim(velocity, sim_time);
        if (trajectory_sim.hasCollisionWith(Eigen::Vector2d(obstacle.x, obstacle.y), robot_radius_))
        {
          velocity_space_.makeInvalidAt(*iterator);
          pruned_velocity_indices_.push_back(*iterator);
          break;
        }
      }
    }  // pointcloud loop ends
  }    // velocity loop ends
}

void DwaPlanner::recoverPrunedVelocity()
{
  if (pruned_velocity_indices_.size() == 0)
    return;

  for (const auto& index : pruned_velocity_indices_)
  {
    velocity_space_.makeValidAt(index);
  }
  pruned_velocity_indices_.clear();
}

void DwaPlanner::maximizeObjectiveFunction()
{
  updateTargetHeading();

  updateClearance();  // fix this: make it more efficient

  updateVelocity();

  updateTargetDistance();

  updateObjectiveFunction();
}

void DwaPlanner::updateTargetHeading()
{
  auto& target_heading_cost = velocity_space_["target_heading"];
  for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
  {
    const int i = iterator.getLinearIndex();
    if (velocity_space_.isInvalidAt(*iterator))
    {
      target_heading_cost(i) = 0;
      continue;
    }

    TrajectorySimulator trajectory_sim(velocity_space_.getVelocityAt(*iterator), collisionCheck_timehorizon_);
    const auto& predicted_trajectory = trajectory_sim.getTrajectory();
    if (predicted_trajectory.size() < 2)
    {
      target_heading_cost(i) = 0;
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
    target_heading_cost(i) = heading_error_scaled;
  }
}

void DwaPlanner::updateClearance()
{
  const auto& validityCheck_matrix = velocity_space_["is_valid"];
  auto& clearance_matrix = velocity_space_["clearance"];

  auto n_row = validityCheck_matrix.rows();
  auto n_col = validityCheck_matrix.cols();

  clearance_matrix = validityCheck_matrix;

  int add_clearance = 10;

  // Expand invalid velocity space downwards (decreased linear velocity near invalid)
  // Shift validity check matrix to downwards
  Eigen::MatrixXf shifted_matrix(n_row, n_col);
  shifted_matrix.block(add_clearance, 0, n_row - add_clearance, n_col) =
      validityCheck_matrix.block(0, 0, n_row - add_clearance, n_col);
  shifted_matrix.block(0, 0, add_clearance, n_col) = clearance_matrix.block(0, 0, add_clearance, n_col);

  for (size_t i = 0; i < add_clearance; ++i)
  {
    shifted_matrix.row(i) = validityCheck_matrix.row(0);
  }

  clearance_matrix = shifted_matrix;
}

void DwaPlanner::updateVelocity()
{
  auto& velocity_matrix = velocity_space_["velocity"];
  velocity_matrix = velocity_space_["v"].cwiseProduct(velocity_space_["is_valid"]);

  // Scale to [0, 1]
  velocity_matrix = velocity_matrix / velocity_matrix.maxCoeff();
}

void DwaPlanner::updateTargetDistance()
{
  auto& target_distance_matrix = velocity_space_["target_distance"];
  for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
  {
    const int i = iterator.getLinearIndex();
    if (velocity_space_.isInvalidAt(*iterator))
    {
      target_distance_matrix(i) = -1;
      continue;
    }

    TrajectorySimulator trajectory_sim(velocity_space_.getVelocityAt(*iterator), collisionCheck_timehorizon_);
    const auto& predicted_trajectory = trajectory_sim.getTrajectory();
    if (predicted_trajectory.empty())  // if trajectory simulation is not valid
    {
      target_distance_matrix(i) = -1;
      continue;
    }

    // Compute predicted distance to target
    double min_distance_to_target = std::numeric_limits<double>::max();
    for (const auto& predicted_position : predicted_trajectory)
    {
      const auto& curr_distance_to_target = (predicted_position - goal_position_).norm();
      min_distance_to_target = std::min(min_distance_to_target, curr_distance_to_target);
    }
    target_distance_matrix(i) = min_distance_to_target;
  }

  auto max_coeff = target_distance_matrix.maxCoeff();
  // Replace negative values (-1) with the maximum value using Eigen's array-wise operations
  target_distance_matrix = (target_distance_matrix.array() < 0).select(max_coeff, target_distance_matrix);
  auto min_coeff = target_distance_matrix.minCoeff();  // after replacing negative values

  // Scale to [0, 1]
  auto max_coeff_matrix =
      Eigen::MatrixXf::Constant(target_distance_matrix.rows(), target_distance_matrix.cols(), max_coeff);
  target_distance_matrix = (max_coeff_matrix - target_distance_matrix) / (max_coeff - min_coeff);
}

void DwaPlanner::updateObjectiveFunction()
{
  const auto& targetHeading = velocity_space_["target_heading"];
  const auto& clearance = velocity_space_["clearance"];
  const auto& velocity = velocity_space_["velocity"];
  const auto& targetDistance = velocity_space_["target_distance"];

  auto& objective_function_matrix = velocity_space_["objective_function"];

  // When goal is backside of the robot, use only target heading and clearance
  // Expected behavior: Rotation in the current position
  if (goal_position_.x() < 0)
  {
    objective_function_matrix =
        weight_targetHeading_ * targetHeading + weight_clearance_ * clearance + weight_targetDistance_ * targetDistance;
  }
  else
  {
    objective_function_matrix = weight_targetHeading_ * targetHeading + weight_clearance_ * clearance +
                                weight_velocity_ * velocity + weight_targetDistance_ * targetDistance;
  }

  // Scale to [0, 1]
  auto min_coeff = objective_function_matrix.minCoeff();
  auto max_coeff = objective_function_matrix.maxCoeff();
  auto min_coeff_matrix =
      Eigen::MatrixXf::Constant(objective_function_matrix.rows(), objective_function_matrix.cols(), min_coeff);
  objective_function_matrix = (objective_function_matrix - min_coeff_matrix) / (max_coeff - min_coeff);
}

Eigen::Vector2d DwaPlanner::findBestVelocity() const
{
  const auto& objective_function_matrix = velocity_space_["objective_function"];

  float max_coeff = 0;
  grid_map::Index index_of_maxCoeff;
  for (grid_map::GridMapIterator iterator(velocity_space_); !iterator.isPastEnd(); ++iterator)
  {
    const auto& i = iterator.getLinearIndex();
    if (objective_function_matrix(i) >= max_coeff)
    {
      max_coeff = objective_function_matrix(i);
      index_of_maxCoeff = *iterator;
    }
  }

  return velocity_space_.getVelocityAt(index_of_maxCoeff);
}

const VelocityWindow& DwaPlanner::getVelocityWindow() const
{
  return velocity_space_;
}