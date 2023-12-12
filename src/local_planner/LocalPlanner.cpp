/*
 * LocalPlanner.cpp
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "local_planner/LocalPlanner.h"
#include "execution_timer/ExecutionTimer.h"
#include "dwa_planner/TrajectorySimulator.h"

namespace ros
{
LocalPlanner::LocalPlanner()
{
  // Pruning of velocity search space by robot kinodynamics
  if (has_max_wheel_velocity.param())
    dwa_planner_.setVelocityspaceLimit(max_wheel_vel.param());

  dwa_planner_.setRobotRadius(robot_radius.param());

  dwa_planner_.setCostWeight(weight_targetHeading.param(), weight_clearance.param(), weight_velocity.param(),
                             weight_targetDistance.param());

  window_visualization_timer.start();
}

void LocalPlanner::localGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  goal_ = *msg;
  velocity_publish_timer.start();
}

void LocalPlanner::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  auto cloud_raw = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *cloud_raw);

  bool has_transformToBase;
  auto base_cloud = pc_processor_.transformPointcloud(cloud_raw, base_frameId.param(), has_transformToBase);
  if (!has_transformToBase)
    return;

  auto height_filtered_cloud = pc_processor_.filterPointcloudByAxis(base_cloud, "z", 0.1, 0.7);
  auto y_filtered_cloud = pc_processor_.filterPointcloudByAxis(height_filtered_cloud, "y", -4.0, 4.0);
  auto downsampled_cloud = pc_processor_.filterPointcloudByVoxel(y_filtered_cloud, 0.2);

  auto local_obstacle_cloud = pc_processor_.filterPointcloudByRange(downsampled_cloud, 0, 5.0);

  // for debug purpose
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*local_obstacle_cloud, cloud_msg);
  local_cloud_publisher.publish(cloud_msg);

  dwa_planner_.doVelocityPruning(local_obstacle_cloud);

  grid_map_msgs::GridMap msg_velocityspace;
  grid_map::GridMapRosConverter::toMessage(dwa_planner_.getVelocityWindow(), msg_velocityspace);
  velocity_window_publisher.publish(msg_velocityspace);
}

void LocalPlanner::DWA(const ros::TimerEvent& event)
{
  // Since the robot is keep moving, goal position in local frame should be updated
  // Goal should be transformed to local(robot) frame first. Then fed into the local planner
  geometry_msgs::PoseStamped local_goal_pose;
  if (!transform_handler_.doTransform(goal_, base_frameId.param(), local_goal_pose))
    return;

  dwa_planner_.setLocalGoal(Eigen::Vector2d(local_goal_pose.pose.position.x, local_goal_pose.pose.position.y));

  dwa_planner_.maximizeObjectiveFunction();

  auto velocity_bset = dwa_planner_.findBestVelocity();

  geometry_msgs::Twist msg_vel;
  DwaPlannerRosConverter::toVelocityMsg(velocity_bset, msg_vel);
  cmdvel_publisher.publish(msg_vel);

  visualization_msgs::Marker msg_path;
  DwaPlannerRosConverter::toPathMsg(velocity_bset, 3.0, msg_path);
  msg_path.color.b = 1.0;
  msg_path.scale.x = 0.05;
  path_publisher.publish(msg_path);
}

// This will be moved to visualization node
// objective functions and path candidates should be visualized in visualization node
// local planner node only visualizes current best local plan and cmd_vel(not visualizable)
void LocalPlanner::visualizeVelocityWindow(const ros::TimerEvent& event)
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
    grid_map::Index index(max_v_index.x() + 1, i);  // almost max linear vel
    auto velocity = v_window.getVelocityAt(index);
    visualization_msgs::Marker path_msg;
    if (v_window.isInvalidAt(index))
    {
      DwaPlannerRosConverter::toPathMsg(velocity, 3.0, path_msg, true);
      path_msg.id = i;
      msg.markers.push_back(path_msg);
    }
    else
    {
      DwaPlannerRosConverter::toPathMsg(velocity, 3.0, path_msg, false);
      path_msg.id = i;
      msg.markers.push_back(path_msg);
    }
  }

  path_candidate_publisher.publish(msg);
}

}  // namespace ros