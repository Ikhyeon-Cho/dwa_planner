#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <ros_node_utils/Core.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_check_test_node");
  ros::NodeHandle nh("~");

  double v = -1.0;
  double w = 0.12;

  double obstacle_x = 1.0;
  double obstacle_y = 1.0;

  double robot_radius = 150;
  std::vector<Eigen::Vector2d> trajectory;
  bool has_collision{ false };

  // left turn
  if (w > 1e-2)
  {
    // for every obstacle points
    if (obstacle_y < 0)
    {
      // pass
    }
    else
    {
      double instance_radius = v / w;

      double instance_center_x = 0;
      double instance_center_y = instance_radius;
      double distance_to_obstacle =
          std::sqrt(std::pow(obstacle_x - instance_center_x, 2) + std::pow((obstacle_y - instance_center_y), 2));

      if (std::abs(distance_to_obstacle - instance_radius) > robot_radius)
      {
        // skip: no collision occur
      }
      else
      {
        // forward sim
        double dt = 0.05;
        Eigen::Vector3d pose(0, 0, 0);
        for (double t = 0; t < 2.0; t += dt)
        {
          double dx = v * dt * std::cos(pose.z());
          double dy = v * dt * std::sin(pose.z());
          double dtheta = w * dt;

          pose.x() += dx;
          pose.y() += dy;
          pose.z() += dtheta;

          trajectory.push_back(pose.head(2));
        }  // trajectory ends

        // Check collision
        std::reverse(trajectory.begin(), trajectory.end());
        for (const auto& pose : trajectory)
        {
          if ((pose - Eigen::Vector2d(obstacle_x, obstacle_y)).norm() < robot_radius)
          {
            has_collision = true;
            break;
          }
        }

        //
      }
    }  // obstacle in same direction ends

  }  // left motion ends

  // right turn
  else if (w < -1e-2)
  {
  }

  // straight
  else
  {
    double predicted_pose_x = v * 2.0;
    if (std::abs(obstacle_x) > predicted_pose_x)
    {
      // pass
    }
  }

  roscpp::Publisher<visualization_msgs::Marker> sim_path{ "path" };
  roscpp::Publisher<geometry_msgs::PoseStamped> obstacle{ "obstacle" };

  visualization_msgs::Marker msg_path;
  geometry_msgs::PoseStamped msg_obstacle;
  msg_obstacle.header.frame_id = "base_link";
  msg_obstacle.pose.position.x = obstacle_x;
  msg_obstacle.pose.position.y = obstacle_y;

  msg_path.header.frame_id = "base_link";
  msg_path.header.stamp = ros::Time::now();
  msg_path.ns = "path";
  msg_path.id = 0;
  msg_path.lifetime = ros::Duration();
  msg_path.action = visualization_msgs::Marker::ADD;
  msg_path.type = visualization_msgs::Marker::LINE_STRIP;

  // Set marker scale
  msg_path.scale.x = 0.05;  // width of the line, or size of points

  // Set msg_path color
  if (has_collision)
  {
    msg_path.color.r = 1.0;
    msg_path.color.g = 0.0;
    msg_path.color.b = 0.0;
    msg_path.color.a = 1.0;  // Don't forget to set the alpha!
  }
  else
  {
    msg_path.color.r = 0.0;
    msg_path.color.g = 1.0;
    msg_path.color.b = 0.0;
    msg_path.color.a = 1.0;  // Don't forget to set the alpha!
  }

  // Fill up the points in the marker
  for (const auto& pose : trajectory)
  {
    geometry_msgs::Point p;
    p.x = pose.x();
    p.y = pose.y();
    msg_path.points.push_back(p);
  }

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    // publish
    obstacle.publish(msg_obstacle);
    sim_path.publish(msg_path);
    loop_rate.sleep();
  }

  return 0;
}