// Copyright 2025 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POINTCLOUD_CROP_BOX_POINTCLOUD_CROP_BOX_POINTCLOUD_CROP_BOX_NODE_HPP_
#define POINTCLOUD_CROP_BOX_POINTCLOUD_CROP_BOX_POINTCLOUD_CROP_BOX_NODE_HPP_

#include <memory>
#include <string>

#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_ros/pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pointcloud_crop_box/pointcloud_crop_box_params.hpp"

namespace pointcloud_crop_box
{

class PointCloudCropBoxNode : public rclcpp::Node
{
public:
  PointCloudCropBoxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void LaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  geometry_msgs::msg::TransformStamped GetTransform(const std::string & source_frame);

  pcl::PointCloud<pcl::PointXYZ>::Ptr TransformCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const geometry_msgs::msg::TransformStamped & transform_stamped);

  pcl::PointCloud<pcl::PointXYZ>::Ptr Crop(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  geometry_msgs::msg::TransformStamped InverseTransform(
    const geometry_msgs::msg::TransformStamped & transform_stamped);

  visualization_msgs::msg::MarkerArray CreateVisualizationMarkers();

  std::shared_ptr<pointcloud_crop_box_params::ParamListener> param_listener_;
  pointcloud_crop_box_params::Params params_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ls_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  laser_geometry::LaserProjection projector_;
};

}  // namespace pointcloud_crop_box

#endif  // POINTCLOUD_CROP_BOX_POINTCLOUD_CROP_BOX_POINTCLOUD_CROP_BOX_NODE_HPP_
