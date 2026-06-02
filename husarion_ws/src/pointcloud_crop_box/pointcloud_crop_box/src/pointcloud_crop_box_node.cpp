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

#include "pointcloud_crop_box/pointcloud_crop_box_node.hpp"

#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pointcloud_crop_box
{

PointCloudCropBoxNode::PointCloudCropBoxNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_crop_box", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  param_listener_ =
    std::make_shared<pointcloud_crop_box_params::ParamListener>(get_node_parameters_interface());

  params_ = param_listener_->get_params();

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(params_.output_topic, 10);
  if (params_.message_type == "pointcloud") {
    pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.input_topic, 10,
      std::bind(&PointCloudCropBoxNode::PointCloudCallback, this, std::placeholders::_1));
  } else {
    ls_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      params_.input_topic, 10,
      std::bind(&PointCloudCropBoxNode::LaserScanCallback, this, std::placeholders::_1));
  }

  if (params_.visualize_crop_boxes) {
    RCLCPP_INFO(this->get_logger(), "Crop boxes visualization enabled.");
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/filter_crop_boxes", 10);
  } else {
    RCLCPP_INFO(this->get_logger(), "Crop boxes visualization disabled.");
  }

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void PointCloudCropBoxNode::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  geometry_msgs::msg::TransformStamped transform_stamped = GetTransform(msg->header.frame_id);

  if (transform_stamped.header.frame_id.empty()) {
    RCLCPP_WARN_STREAM(
      this->get_logger(), "No transform found for frame: " << msg->header.frame_id);
    return;
  }

  pcl::fromROSMsg(*msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = TransformCloud(cloud, transform_stamped);
  if (!transformed_cloud) {
    RCLCPP_WARN(this->get_logger(), "Cloud transformation failed.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = Crop(transformed_cloud);
  if (!filtered_cloud) {
    RCLCPP_WARN(this->get_logger(), "Crop box filtering failed.");
    return;
  }

  geometry_msgs::msg::TransformStamped inverse_transform_stamped =
    InverseTransform(transform_stamped);
  pcl_ros::transformPointCloud(*filtered_cloud, *filtered_cloud, inverse_transform_stamped);

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*filtered_cloud, output_msg);
  output_msg.header = msg->header;

  pub_->publish(output_msg);

  if (params_.visualize_crop_boxes) {
    auto marker_array = CreateVisualizationMarkers();
    marker_pub_->publish(marker_array);
  }
}

void PointCloudCropBoxNode::LaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

  projector_.projectLaser(*msg, *cloud_msg);
  PointCloudCallback(cloud_msg);
}

geometry_msgs::msg::TransformStamped PointCloudCropBoxNode::GetTransform(
  const std::string & source_frame)
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped = tf_buffer_.lookupTransform(
      params_.target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(
      this->get_logger(), "Could not transform " << params_.target_frame << " to " << source_frame
                                                 << ": " << ex.what());
  }

  return transform_stamped;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudCropBoxNode::TransformCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  try {
    pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transform_stamped);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Error during cloud transformation: %s", ex.what());
    return nullptr;
  }

  return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudCropBoxNode::Crop(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  crop_box_filter.setInputCloud(cloud);
  for (auto const & [key, val] : params_.crop_boxes.crop_boxes_names_map) {
    crop_box_filter.setMin(Eigen::Vector4f(val.min_x, val.min_y, val.min_z, 1.0));
    crop_box_filter.setMax(Eigen::Vector4f(val.max_x, val.max_y, val.max_z, 1.0));
    crop_box_filter.setNegative(params_.negative);
    crop_box_filter.filter(*filtered_cloud);
    crop_box_filter.setInputCloud(filtered_cloud);
  }
  return filtered_cloud;
}

geometry_msgs::msg::TransformStamped PointCloudCropBoxNode::InverseTransform(
  const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  tf2::Transform tf2_transform;
  tf2::fromMsg(transform_stamped.transform, tf2_transform);

  tf2::Transform tf2_inverse = tf2_transform.inverse();

  geometry_msgs::msg::TransformStamped inverse_transform_stamped;
  inverse_transform_stamped.header.stamp = transform_stamped.header.stamp;
  inverse_transform_stamped.header.frame_id = transform_stamped.child_frame_id;
  inverse_transform_stamped.child_frame_id = transform_stamped.header.frame_id;

  geometry_msgs::msg::Transform geom_transform;
  geom_transform.translation.x = tf2_inverse.getOrigin().x();
  geom_transform.translation.y = tf2_inverse.getOrigin().y();
  geom_transform.translation.z = tf2_inverse.getOrigin().z();

  tf2::Quaternion quat = tf2_inverse.getRotation();
  geom_transform.rotation.x = quat.x();
  geom_transform.rotation.y = quat.y();
  geom_transform.rotation.z = quat.z();
  geom_transform.rotation.w = quat.w();

  inverse_transform_stamped.transform = geom_transform;

  return inverse_transform_stamped;
}

visualization_msgs::msg::MarkerArray PointCloudCropBoxNode::CreateVisualizationMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;
  for (const auto & [key, val] : params_.crop_boxes.crop_boxes_names_map) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.target_frame;
    marker.header.stamp = this->now();
    marker.ns = key;
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;  // Line width
    marker.color.a = 1.0;   // Fully opaque
    marker.color.r = 1.0;   // Red color

    // Define the 8 points of the bounding box
    std::array<Eigen::Vector3f, 8> points = {
      Eigen::Vector3f(val.min_x, val.min_y, val.min_z),
      Eigen::Vector3f(val.max_x, val.min_y, val.min_z),
      Eigen::Vector3f(val.max_x, val.max_y, val.min_z),
      Eigen::Vector3f(val.min_x, val.max_y, val.min_z),
      Eigen::Vector3f(val.min_x, val.min_y, val.max_z),
      Eigen::Vector3f(val.max_x, val.min_y, val.max_z),
      Eigen::Vector3f(val.max_x, val.max_y, val.max_z),
      Eigen::Vector3f(val.min_x, val.max_y, val.max_z),
    };

    // Define the lines that make up the bounding box
    std::vector<std::pair<int, int>> line_indices = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
      {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
    };

    for (const auto & [start, end] : line_indices) {
      geometry_msgs::msg::Point p_start;
      p_start.x = points[start].x();
      p_start.y = points[start].y();
      p_start.z = points[start].z();

      geometry_msgs::msg::Point p_end;
      p_end.x = points[end].x();
      p_end.y = points[end].y();
      p_end.z = points[end].z();

      marker.points.push_back(p_start);
      marker.points.push_back(p_end);
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

}  // namespace pointcloud_crop_box

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_crop_box::PointCloudCropBoxNode)
