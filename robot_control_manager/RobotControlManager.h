/*
 * RobotControlManager.h
 *
 *  Created on: Dec 13, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROBOT_CONTROL_MANAGER_H
#define ROBOT_CONTROL_MANAGER_H

#include <ros/ros.h>
#include <ros_node_utils/Core.h>
#include <ros_transform_utils/TransformHandler.h>

#include <geometry_msgs/Twist.h>

namespace ros
{

class RobotControlManager
{
public:
  RobotControlManager();

  void savePlannedVelocity(const geometry_msgs::TwistConstPtr& msg);

  void saveGlobalGoal(const geometry_msgs::PoseStampedConstPtr& msg);

  void checkGoalAchievement(const ros::TimerEvent& event);

public:
  // Subscribed Topics
  roscpp::Parameter<std::string> plannedVelocity_topic{ "~/Subscribed_Topics/planned_velocity", "/best_velocity" };
  roscpp::Parameter<std::string> globalGoal_topic{ "~/Subscribed_Topics/global_goal", "/move_base_simple/goal" };

  // Published Topics
  roscpp::Parameter<std::string> commandVelocity_topic{ "~/Published_Topics/command_velocity", "/cmd_vel" };

  // Parameters
  // -- Frame Ids
  roscpp::Parameter<std::string> base_frameId{ "~/Parameters/base_frameId", "base_link" };
  roscpp::Parameter<std::string> map_frameId{ "~/Parameters/map_frameId", "map" };
  // -- Duration
  roscpp::Parameter<double> poseUpdate_duration{ "~/Parameters/pose_update_duration", 0.05 };
  // -- Status check: Goal acheivement
  roscpp::Parameter<double> goal_tolerance{ "~/Parameters/goal_tolerance", 0.3 };

private:
  TransformHandler transform_handler_;

  geometry_msgs::Twist planned_velocity_;
  geometry_msgs::Twist command_velocity_;
  geometry_msgs::PoseStamped global_goal_;

  roscpp::Subscriber<geometry_msgs::Twist> plannedVelocity_subscriber{ plannedVelocity_topic.param(),
                                                                       &RobotControlManager::savePlannedVelocity,
                                                                       this };
  roscpp::Subscriber<geometry_msgs::PoseStamped> globalGoal_subscriber{ globalGoal_topic.param(),
                                                                        &RobotControlManager::saveGlobalGoal, this };

  roscpp::Timer poseUpdate_timer{ poseUpdate_duration.param(), &RobotControlManager::checkGoalAchievement, this };

  roscpp::Publisher<geometry_msgs::Twist> commandVelocity_publisher{ commandVelocity_topic.param() };
};

RobotControlManager::RobotControlManager()
{
}

void RobotControlManager::savePlannedVelocity(const geometry_msgs::TwistConstPtr& msg)
{
  planned_velocity_ = *msg;
}

void RobotControlManager::saveGlobalGoal(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // Set goal position in map frame
  if (!transform_handler_.doTransform(*msg, map_frameId.param(), global_goal_))
    return;

  poseUpdate_timer.start();
}

void RobotControlManager::checkGoalAchievement(const ros::TimerEvent& event)
{
  // get robot position
  geometry_msgs::TransformStamped robot_pose;
  if (!transform_handler_.getTransform(map_frameId.param(), base_frameId.param(), robot_pose))
    return;
  Eigen::Vector2d robot_position(robot_pose.transform.translation.x, robot_pose.transform.translation.y);

  // get goal position
  Eigen::Vector2d goal_position(global_goal_.pose.position.x, global_goal_.pose.position.y);

  if ((robot_position - goal_position).norm() < goal_tolerance.param())
  {
    commandVelocity_publisher.publish(geometry_msgs::Twist());
  }
}

}  // namespace ros

#endif  // ROBOT_CONTROL_MANAGER_H