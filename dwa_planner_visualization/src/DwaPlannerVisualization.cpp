#include "dwa_planner_visualization/DwaPlannerVisualization.h"

DwaPlannerVisualization::DwaPlannerVisualization()
{
}

void DwaPlannerVisualization::velocityWindowCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, velocity_window_);
}

void DwaPlannerVisualization::visualize(const ros::TimerEvent& event)
{
  visualization_msgs::Marker msg;

  // fill the msg

  path_publisher_.publish(msg);
}