/*
 * pointcloud.h
 *
 *  Created on: Apr 22, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_UTILS_POINTCLOUD_H
#define ROS_UTILS_POINTCLOUD_H

// PCL
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// TF Transform
#include "transform.h"

template <typename T>
using PointCloud = pcl::PointCloud<T>;

template <typename T>
using PointCloudPtr = typename PointCloud<T>::Ptr;

namespace ros_utils
{
namespace pcl
{
/// @tparam T point type of the pointcloud
/// @param input The pointcloud to be transformed
/// @param transform_stamped The transform to be applied
/// @return Transformed pointcloud (If fails, then returns input pointcloud)
template <typename T>
static PointCloudPtr<T> transformPointcloud(const PointCloudPtr<T>& input,
                                            const geometry_msgs::TransformStamped& transform_stamped)
{
  std::string source_frame(input->header.frame_id);
  if (source_frame.empty())
  {
    ROS_ERROR_STREAM_THROTTLE(3, " [ros_utils::pcl] Warning: Transform failure -  pointcloud has no frame id");
    return input;
  }

  if (input->empty())
  {
    ROS_ERROR_STREAM_THROTTLE(3," [ros_utils::pcl] Warning: Transform failure -  pointcloud is empty");
    return input;
  }

  auto transform_affine3d = ros_utils::tf::toAffine3d(transform_stamped.transform);

  auto output = boost::make_shared<PointCloud<T>>();
  ::pcl::transformPointCloud(*input, *output, transform_affine3d);
  output->header = input->header;
  output->header.frame_id = transform_stamped.header.frame_id;
  return output;
}

/// @brief
/// @tparam T point type of the pointcloud
/// @param input The pointcloud to be filtered
/// @param field The field name to be filtered
/// @param field_min Lower bound of the filter
/// @param field_max  Upper bound of the filter
/// @param negative If true, then the points outside [min, max] will remain (default: false)
/// @return Filtered pointcloud
template <typename T>
static PointCloudPtr<T> filterPointcloudByField(const PointCloudPtr<T>& input, const std::string& field,
                                                double field_min, double field_max, bool negative = false)
{
  if (input->empty())
    return input;

  PointCloudPtr<T> output = boost::make_shared<PointCloud<T>>();

  ::pcl::PassThrough<T> ps;
  ps.setInputCloud(input);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(field_min, field_max);
  ps.setNegative(negative);
  ps.filter(*output);
  output->header = input->header;
  return output;
}

template <typename T>
static PointCloudPtr<T> filterPointcloudByRange2D(const PointCloudPtr<T>& input, double range_min, double range_max)
{
  if (input->empty())
    return input;

  PointCloudPtr<T> output = boost::make_shared<PointCloud<T>>();
  for (auto point : input->points)
  {
    double range_2D = sqrt(point.x * point.x + point.y * point.y);

    if (range_2D > range_min && range_2D < range_max)
    {
      T filtered_point = point;
      output->points.push_back(filtered_point);
    }
  }
  output->header = input->header;
  return output;
}

template <typename T>
static PointCloudPtr<T> filterPointcloudByRange(const PointCloudPtr<T>& input, double range_min, double range_max)
{
  if (input->empty())
    return input;

  PointCloudPtr<T> output = boost::make_shared<PointCloud<T>>();
  for (auto point : input->points)
  {
    double range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    if (range > range_min && range < range_max)
    {
      T filtered_point = point;
      output->points.push_back(filtered_point);
    }
  }
  output->header = input->header;
  return output;
}

template <typename T>
static PointCloudPtr<T> filterPointcloudByAngle(const PointCloudPtr<T>& input, double angle_start, double angle_end,
                                                bool negative = false)
{
  if (input->empty())
    return input;

  PointCloudPtr<T> output = boost::make_shared<PointCloud<T>>();
  for (auto point : input->points)
  {
    double angle_horizon = std::atan2(point.y, point.x);
    bool filter_condition = angle_horizon > DEG2RAD(angle_start) && angle_horizon < DEG2RAD(angle_end);
    if (negative)
      filter_condition = !filter_condition;

    if (filter_condition)
    {
      T filtered_point = point;
      filtered_point.range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      output->points.push_back(filtered_point);
    }
  }
  output->header = input->header;
  return output;
}

template <typename T>
static PointCloudPtr<T> filterPointcloudByVoxel(const PointCloudPtr<T>& input, double voxel_x, double voxel_y,
                                                double voxel_z)
{
  if (input->empty())
    return input;

  PointCloudPtr<T> output = boost::make_shared<PointCloud<T>>();
  ::pcl::VoxelGrid<T> vox;
  vox.setInputCloud(input);
  vox.setLeafSize(voxel_x, voxel_y, voxel_z);
  vox.filter(*output);
  output->header = input->header;
  return output;
}

template <typename T>
static PointCloudPtr<T> filterPointcloudByVoxel(const PointCloudPtr<T>& input, double voxel_size)
{
  return filterPointcloudByVoxel(input, voxel_size, voxel_size, voxel_size);
}

}  // namespace pcl
};  // namespace ros_utils

#endif  // ROS_UTILS_POINTCLOUD_H