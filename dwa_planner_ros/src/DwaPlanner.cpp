/*
 * DwaPlanner.cpp
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "dwa_planner_ros/DwaPlanner.h"
// #include "execution_timer/ExecutionTimer.h"
#include "dwa_planner_core/TrajectorySimulator.h"

DwaPlanner::DwaPlanner()
{
  dwa_planner_.setCollsionCheckParams(robot_radius_, collision_check_timehorizon_, safety_margin_);
  dwa_planner_.setCostParam(optimization_timehorizon_, weight_targetheading_, weight_clearance_, weight_velocity_);

  // Prune velocity space by using robot kinematics
  dwa_planner_.setVelocityspaceLimit(max_linear_vel_, wheelbase_length_);
}

void DwaPlanner::localGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  goal_ = *msg;
  local_plan_timer_.start();
}

void DwaPlanner::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Convert msg to pcl pointcloud
  auto cloud_raw = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *cloud_raw);
  const auto& sensor_frame = msg->header.frame_id;

  // Transform pointcloud to base frame
  auto [has_transform_s2b, sensor_to_base] = tf_handler_.getTransform(sensor_frame, baselink_frame_);
  if (!has_transform_s2b)
    return;

  auto cloud_base = ros_utils::pcl::transformPointcloud<pcl::PointXYZI>(cloud_raw, sensor_to_base);

  // Filter pointcloud
  auto cloud_filtered = ros_utils::pcl::filterPointcloudByField<pcl::PointXYZI>(cloud_base, "z", 0.1, 0.7);
  cloud_filtered = ros_utils::pcl::filterPointcloudByField<pcl::PointXYZI>(cloud_filtered, "y", -4.0, 4.0);
  auto cloud_downsampled = ros_utils::pcl::filterPointcloudByVoxel<pcl::PointXYZI>(cloud_filtered, 0.2, 0.2, 0.2);
  
  // filterPointcloudByVoxel<pcl::PointXYZI>(cloud_filtered, 0.2);

  auto local_obstacle = ros_utils::pcl::filterPointcloudByRange<pcl::PointXYZI>(cloud_downsampled, 0, 5.0);

  // Prune velocity space by using local obstacle information
  dwa_planner_.doVelocityPruning(local_obstacle);

  // Visualize obstacles: For Debug purpose
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*local_obstacle, cloud_msg);
  pub_obstacle_cloud_.publish(cloud_msg);

  // Visualize velocity window: For Debug purpose
  grid_map_msgs::GridMap msg_velocityspace;
  grid_map::GridMapRosConverter::toMessage(dwa_planner_.getVelocityWindow(), msg_velocityspace);
  pub_velocity_window_.publish(msg_velocityspace);
}

void DwaPlanner::DWA(const ros::TimerEvent& event)
{
  // Since the robot is keep moving, goal position in local frame should be updated
  // Goal should be transformed to local(robot) frame first. Then fed into the local planner
  auto [has_transform_m2b, map_to_base] = tf_handler_.getTransform(map_frame_, baselink_frame_);
  if (!has_transform_m2b)
    return;

  geometry_msgs::PoseStamped local_goal_pose;
  if (!ros_utils::tf::doTransform(goal_, local_goal_pose, map_to_base))
    return;

  dwa_planner_.setLocalGoal(Eigen::Vector2d(local_goal_pose.pose.position.x, local_goal_pose.pose.position.y));

  dwa_planner_.maximizeObjectiveFunction();

  auto velocity_bset = dwa_planner_.findBestVelocity();

  geometry_msgs::Twist msg_vel;
  DwaMsgs::toVelocityMsg(velocity_bset, msg_vel);
  pub_velocity_.publish(msg_vel);

  visualization_msgs::Marker msg_path;
  DwaMsgs::toPathMsg(velocity_bset, 3.0, msg_path);
  msg_path.color.b = 1.0;
  msg_path.scale.x = 0.05;
  pub_path_best_.publish(msg_path);
}

// This will be moved to visualization node
// objective functions and path candidates should be visualized in visualization node
// local planner node only visualizes current best local plan and cmd_vel(not visualizable)
void DwaPlanner::visualizeVelocityWindow(const ros::TimerEvent& event)
{
  const auto& v_window = dwa_planner_.getVelocityWindow();
  double v_max = 0;
  grid_map::Index max_v_index;

  std::vector<Eigen::Vector2d> velocity_candidates;
  int i = 0;
  int subsample_interval = 2;
  for (grid_map::GridMapIterator iterator(v_window); !iterator.isPastEnd(); ++iterator)
  {
    i += 1;

    const auto& index = *iterator;
    if (v_window.isInvalidAt(index))
      continue;

    if (i % subsample_interval != 0)
      continue;

    auto velocity = v_window.getVelocityAt(*iterator);
    if (velocity.x() > v_max)
    {
      v_max = velocity.x();
      max_v_index = *iterator;
    }
  }

  visualization_msgs::MarkerArray msg;
  // iteration to column-wise direction
  for (size_t i = 0; i < v_window.getSize()(1); ++i)
  {
    grid_map::Index index(max_v_index.x() + 4, i);  // almost max linear vel
    auto velocity = v_window.getVelocityAt(index);
    visualization_msgs::Marker path_msg;
    if (v_window.isInvalidAt(index))
    {
      DwaMsgs::toPathMsg(velocity, optimization_timehorizon_, path_msg, true);
      path_msg.id = i;
      msg.markers.push_back(path_msg);
    }
    else
    {
      DwaMsgs::toPathMsg(velocity, optimization_timehorizon_, path_msg, false);
      path_msg.id = i;
      msg.markers.push_back(path_msg);
    }
  }

  pub_path_candidates_.publish(msg);
}