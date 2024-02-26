/*
 * dwa_planner_visualization_node.cpp
 *
 *  Created on: Nov 30, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "dwa_planner_visualization/DwaPlannerVisualization.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dwa_planner_visualization_node");
  ros::NodeHandle nh("~");

  DwaPlannerVisualization visualization_node;

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}