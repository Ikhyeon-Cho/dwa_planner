/*
 * local_planner_node.cpp
 *
 *  Created on: Sep 3, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "local_planner/LocalPlanner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dwa_planner_node");
  ros::NodeHandle nh("~");

  ros::LocalPlanner node;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}