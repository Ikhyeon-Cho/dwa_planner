#include "local_planner_visualization/LocalPlannerVisualization.h"

namespace ros
{

LocalPlannerVisualization::LocalPlannerVisualization()
{
  vis_timer.start();
}

void LocalPlannerVisualization::velocityWindowCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, velocity_window_);
}

void LocalPlannerVisualization::visualize(const ros::TimerEvent& event)
{
  visualization_msgs::Marker msg;

  // fill the msg
  

  path_publisher.publish(msg);
}

}  // namespace ros
