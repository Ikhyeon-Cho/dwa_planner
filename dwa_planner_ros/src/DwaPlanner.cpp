/*
 * DwaPlanner.cpp
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "dwa_planner_ros/DwaPlanner.h"
#include "trajectory_simulator/TrajectorySimulator.h"

DwaPlanner::DwaPlanner() : dwa_planner_{ robot_max_v_, robot_max_w_, velocity_resolution_, enable_backward_motion_ }
{
  // Assuming Two-wheeled diff robot: need to set wheelbase for kinematic constraints
  // dwa_planner_.setKinematicConstraints(robot_max_v_, wheelbase_);

  // Reference velocity [m/s]
  dwa_planner_.setReferenceVelocity(robot_ref_v_);

  // Collsion check paramters
  dwa_planner_.setRobotSize(robot_radius_);
  dwa_planner_.setTimeHorizon(time_horizon_);

  // Dwa cost parameters
  dwa_planner_.setTargetHeadingWeight(w_targetheading_);
  dwa_planner_.setVelocityWeight(w_velocity_);
  dwa_planner_.setClearanceWeight(w_clearance_);
  dwa_planner_.setTimeHorizon(time_horizon_);

  // Check whether the laser subscriber is ready
  if (!ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pointcloud_topic_, ros::Duration(1)))
    ROS_WARN("%-65s %s", "Laser topic is not ready. DWA is blind!!", "[ DWA Planner]");

  // Visualizations for debug
  if (use_debug_)
  {
    pub_obstacle_cloud_ = nh_priv_.advertise<sensor_msgs::PointCloud2>("debug/pointcloud", 10);
    pub_velocity_window_ = nh_priv_.advertise<grid_map_msgs::GridMap>("debug/window", 10);
    pub_target_heading_plan_ = nh_priv_.advertise<visualization_msgs::Marker>("debug/target_heading_plan", 10);
    pub_clearance_plan_ = nh_priv_.advertise<visualization_msgs::Marker>("debug/clearance_plan", 10);
    pub_velocity_plan_ = nh_priv_.advertise<visualization_msgs::Marker>("debug/velocity_plan", 10);

    ROS_INFO("%-65s %s", "Debug mode is enabled. Publishing dynamic window costs...", "[ DWA Planner]");
    publish_timer_.start();
  }
}

void DwaPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  msg_goal_ = *msg;
  ROS_INFO("%-65s %s", "Received a new goal.", "[ DWA Planner]");

  if (!alignToGoal(msg_goal_))
    return;

  local_plan_timer_.start();  // plan starts when goal is set
  publish_timer_.start();     // publish best local plan
}

bool DwaPlanner::alignToGoal(const geometry_msgs::PoseStamped& goal)
{
  bool is_aligned = false;

  geometry_msgs::PoseStamped local_goal;
  if (!getLocalGoal(goal, local_goal))
  {
    ROS_ERROR("%-65s %s", "Failed to transform goal to robot frame. Are the tf tree correct?", "[ DWA Planner]");
    return false;
  }

  // Check whether the goal is behind the robot
  if (local_goal.pose.position.x > 0)
  {
    is_aligned = true;
    return true;
  }

  ROS_INFO("%-65s %s", "Aligning to the goal...", "[ DWA Planner]");

  while (!is_aligned && ros::ok())
  {
    // Keep aligning the robot to the goal
    getLocalGoal(goal, local_goal);
    const auto& goal_position = local_goal.pose.position;
    dwa_planner_.setGoal(goal_position.x, goal_position.y);
    dwa_planner_.updateWindow();

    Eigen::Vector2d robot_heading_vector(1, 0);
    Eigen::Vector2d target_heading_vector(local_goal.pose.position.x, local_goal.pose.position.y);
    double angle_diff = std::atan2(target_heading_vector.y(), target_heading_vector.x()) -
                        std::atan2(robot_heading_vector.y(), robot_heading_vector.x());

    const auto left_rotation = angle_diff > 1e-4;
    const auto right_rotation = angle_diff < -1e-4;
    if (std::abs(angle_diff) > 0.1 && left_rotation)
    {
      // rotate at left
      geometry_msgs::Twist msg_vel;
      DwaMsgs::toVelocityMsg(Eigen::Vector2d(0, 0.5), msg_vel);
      pub_velocity_.publish(msg_vel);
      ros::spinOnce();
      ros::Rate(velocity_pub_rate_).sleep();
    }
    else if (std::abs(angle_diff) > 0.1 && right_rotation)
    {
      // rotate at right
      geometry_msgs::Twist msg_vel;
      DwaMsgs::toVelocityMsg(Eigen::Vector2d(0, -0.5), msg_vel);
      pub_velocity_.publish(msg_vel);
      ros::spinOnce();
      ros::Rate(velocity_pub_rate_).sleep();
    }
    else
    {
      is_aligned = true;
    }
  }

  return true;
}

void DwaPlanner::plan(const ros::TimerEvent& event)
{
  if (requireEmergencyStop())
  {
    // Stop the robot
    geometry_msgs::Twist msg_vel;
    DwaMsgs::toVelocityMsg(Eigen::Vector2d(0, 0), msg_vel);
    pub_velocity_.publish(msg_vel);
    return;
  }

  // Since the robot is keep moving, goal position in local view should be updated
  // --> Goal should be transformed to local(robot) frame first. Then fed into the local planner
  geometry_msgs::PoseStamped local_goal;
  if (!getLocalGoal(msg_goal_, local_goal))
  {
    ROS_ERROR("%-65s %s", "Failed to transform goal to robot frame. Are the tf tree correct?", "[ DWA Planner]");
    return;
  }

  if (isArrivedAt(local_goal))
  {
    ROS_INFO("%-65s %s", "Arrived at the goal.", "[ DWA Planner]");

    // Stop the planner
    local_plan_timer_.stop();

    // Stop the robot
    geometry_msgs::Twist msg_vel;
    DwaMsgs::toVelocityMsg(Eigen::Vector2d(0, 0), msg_vel);
    pub_velocity_.publish(msg_vel);
    return;
  }

  else
  {
    ROS_INFO_THROTTLE(2, "%-65s %s", "Planning...", "[ DWA Planner]");
    // Update local plan
    const auto& goal_position = local_goal.pose.position;
    dwa_planner_.setGoal(goal_position.x, goal_position.y);
    dwa_planner_.updateWindow();

    auto planned_velocity = dwa_planner_.getBestPlan();
    geometry_msgs::Twist msg_vel;
    DwaMsgs::toVelocityMsg(planned_velocity, msg_vel);
    pub_velocity_.publish(msg_vel);
  }
}

bool DwaPlanner::requireEmergencyStop() const
{
  if (dwa_planner_.hasNoPlan())
  {
    ROS_WARN_THROTTLE(1, "%-65s %s", "No feasible plan. Emergency stop.", "[ DWA Planner]");
    return true;
  }

  if (min_distance_to_obstacle_ < robot_radius_ + safety_margin_)
  {
    ROS_WARN_THROTTLE(1, "%-65s %s", "Too close to the obstacle. Emergency stop.", "[ DWA Planner]");
    return true;
  }
  return false;
}

bool DwaPlanner::isArrivedAt(const geometry_msgs::PoseStamped& goal)
{
  // check the distance to the goal
  const auto& goal_position = goal.pose.position;
  double dist_to_goal = Eigen::Vector2d(goal_position.x, goal_position.y).norm();
  return dist_to_goal < radius_goal_reached_;
}

void DwaPlanner::laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Convert msg to pcl pointcloud
  auto pointcloud_raw = boost::make_shared<pcl::PointCloud<pcl::PointWithRange>>();
  pcl::fromROSMsg(*msg, *pointcloud_raw);

  if (pointcloud_raw->empty())
  {
    ROS_WARN("%-65s %s", "Received an empty pointcloud.", "[ DWA Planner]");
    return;
  }

  // Downsample pointcloud
  auto pointcloud_downsampled =
      ros_utils::pcl::filterPointcloudByVoxel<pcl::PointWithRange>(pointcloud_raw, 0.2, 0.2, 0.2);

  // Transform pointcloud to base frame
  const auto& sensor_frame = msg->header.frame_id;
  auto [has_transform_s2b, tf_sensor_to_base] = tf_.getTransform(sensor_frame, baselink_frame_);
  if (!has_transform_s2b)
    return;
  auto pointcloud_base =
      ros_utils::pcl::transformPointcloud<pcl::PointWithRange>(pointcloud_downsampled, tf_sensor_to_base);

  // Set Local points as obstacles
  obstacle_points_ =
      ros_utils::pcl::filterPointcloudByRange<pcl::PointWithRange>(pointcloud_downsampled, 0, max_laser_range_);
  obstacle_points_ =
      ros_utils::pcl::filterPointcloudByField<pcl::PointWithRange>(obstacle_points_, "x", 0, max_laser_range_);

  if (obstacle_points_->empty())
    return;

  // Prune velocity space by using local obstacle points
  dwa_planner_.updateVelocityObstacles(obstacle_points_);
  min_distance_to_obstacle_ = getMinDistanceToObstacle(obstacle_points_);
}

double DwaPlanner::getMinDistanceToObstacle(const pcl::PointCloud<pcl::PointWithRange>::Ptr& obstacle_points)
{
  double min_distance = 1e+4;
  for (const auto& point : *obstacle_points)
  {
    // double distance = point.range;
    double distance = std::sqrt(point.x * point.x + point.y * point.y);
    min_distance = std::min(min_distance, distance);
  }
  return min_distance;
}

void DwaPlanner::visualizeTrajectories()
{
  const auto& window = dwa_planner_.getVelocityWindow();

  // Visualize the valid path candidates
  visualization_msgs::MarkerArray msg_trajectories;
  for (size_t w_index = 0; w_index < window.getSize()(1); ++w_index)
  {
    auto max_v_index = window.getMaxVelocityIndexAt(w_index);
    grid_map::Index index(max_v_index, w_index);  // Index with max v for each w
    auto [v, w] = window.getVelocityAt(index);

    visualization_msgs::Marker path_msg;
    if (window.isInvalidAt(index))
    {
      DwaMsgs::toPathMsg(Eigen::Vector2d(v, w), time_horizon_, path_msg, "red");
      path_msg.id = i;
      path_msg.color.r = 1.0;
      path_msg.color.g = 0.0;
      path_msg.color.b = 0.0;
      path_msg.color.a = 0.9;
      msg.markers.push_back(path_msg);
    }
    else
    {
      DwaMsgs::toPathMsg(Eigen::Vector2d(v, w), time_horizon_, path_msg, "green");
      path_msg.id = i;
      msg.markers.push_back(path_msg);
    }
  }
  pub_path_candidates_.publish(msg);

  // // Visualize the path candidates around the best plan
  // visualization_msgs::MarkerArray msg_candidates;
  // auto planned_velocity_index = dwa_planner_.getBestPlanVelocityIndex();

  // std::cout << planned_velocity_index << std::endl << std::endl;
  
  // auto planned_v_index = planned_velocity_index.x();
  // auto planned_w_index = planned_velocity_index.y();
  // auto min_idx = std::max(0, planned_v_index - 10);
  // auto max_idx = std::min(window.getSize()(1), planned_v_index + 10);
  // for (size_t i = min_idx; i < max_idx; ++i)
  // {
  //   grid_map::Index index(max_v_index.x(), i);  // max linear vel
  //   auto [v, w] = window.getVelocityAt(index);
  //   visualization_msgs::Marker path_msg;
  //   DwaMsgs::toPathMsg(Eigen::Vector2d(v, w), time_horizon_, path_msg, "blue");
  //   path_msg.id = i;
  //   msg_candidates.markers.push_back(path_msg);
  // }
  // pub_path_around_best_.publish(msg_candidates);
}

void DwaPlanner::publish(const ros::TimerEvent& event)
{
  // Best local plan (blue)
  auto planned_velocity = dwa_planner_.getBestPlan();
  visualization_msgs::Marker msg_path;
  DwaMsgs::toPathMsg(planned_velocity, time_horizon_, msg_path);
  msg_path.color.b = 1.0;  // blue
  msg_path.scale.x = 0.05;
  pub_path_best_.publish(msg_path);

  if (use_debug_)
  {
    // obstacles
    sensor_msgs::PointCloud2 obstacle;
    pcl::toROSMsg(*obstacle_points_, obstacle);
    pub_obstacle_cloud_.publish(obstacle);

    // Cost Window
    auto v_window_visualize = dwa_planner_.getVelocityWindow();
    v_window_visualize["TargetHeading"] *= 0.3;
    v_window_visualize["Clearance"] *= 0.3;
    v_window_visualize["Velocity"] *= 0.3;
    v_window_visualize["TotalCost"] *= 0.3;
    grid_map_msgs::GridMap window;
    grid_map::GridMapRosConverter::toMessage(v_window_visualize, window);
    pub_velocity_window_.publish(window);

    // Target Heading Plan
    auto target_heading_plan = dwa_planner_.getBestPlanAt("TargetHeading");
    visualization_msgs::Marker msg_heading;
    DwaMsgs::toPathMsg(target_heading_plan, time_horizon_, msg_heading);
    // green
    msg_heading.scale.x = 0.05;
    pub_target_heading_plan_.publish(msg_heading);

    // Clearance Plan
    auto clearance_plan = dwa_planner_.getBestPlanAt("Clearance");
    visualization_msgs::Marker msg_clearance;
    DwaMsgs::toPathMsg(clearance_plan, time_horizon_, msg_clearance);
    msg_clearance.color.r = 1.0;  // red
    msg_clearance.scale.x = 0.05;
    pub_clearance_plan_.publish(msg_clearance);
  }
}

bool DwaPlanner::getLocalGoal(const geometry_msgs::PoseStamped& goal, geometry_msgs::PoseStamped& goal_local)
{
  const auto& map_frame = goal.header.frame_id;
  auto [has_transform, tf_to_baselink] = tf_.getTransform(map_frame, baselink_frame_);
  if (!has_transform || !ros_utils::tf::doTransform(goal, goal_local, tf_to_baselink))
    return false;
  return true;
}