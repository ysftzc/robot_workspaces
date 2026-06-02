// Copyright 2024 Husarion sp. z o.o.
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

#include "wibotic_connector_can/wibotic_can_driver_node.hpp"

namespace wibotic_connector_can
{
WiboticCanDriverNode::WiboticCanDriverNode(
  const std::string & node_name, WiboticCanDriverInterface::SharedPtr wibotic_can_driver,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options), wibotic_can_driver_(wibotic_can_driver)

{
  RCLCPP_INFO(this->get_logger(), "Initializing node.");
  if (!wibotic_can_driver_) {
    throw std::runtime_error("Wibotic CAN driver is not initialized.");
  }

  DeclareParameters();
  GetParameters();

  try {
    CreateWiboticCanDriver();
  } catch (const uavcan_linux::Exception & e) {
    RCLCPP_FATAL_STREAM(
      this->get_logger(), "An occurred error during creating WiboticCanDriver: " << e.what());
    throw;
  }

  wibotic_info_pub_ = this->create_publisher<wibotic_msgs::msg::WiboticInfo>("wibotic_info", 10);
  wibotic_charger_enable_service_ = this->create_service<std_srvs::srv::SetBool>(
    "wibotic_charger_enable", std::bind(
                                &WiboticCanDriverNode::WiboticChargerEnableCallback, this,
                                std::placeholders::_1, std::placeholders::_2));

  wibotic_info_timer_ = this->create_wall_timer(
    std::chrono::duration<float>(update_time_),
    std::bind(&WiboticCanDriverNode::WiboticInfoTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Node initialized successfully.");
}

void WiboticCanDriverNode::DeclareParameters()
{
  this->declare_parameter("can_iface_name", "can0");
  this->declare_parameter("uavcan_node_id", 20);
  this->declare_parameter("uavcan_node_name", "com.wibotic.ros_connector");
  this->declare_parameter("update_time", 1.0);
  this->declare_parameter("max_service_call_retries", 10);
  this->declare_parameter("spin_duration", 0.1);
}

void WiboticCanDriverNode::GetParameters()
{
  can_iface_name_ = this->get_parameter("can_iface_name").as_string();
  uavcan_node_id_ = this->get_parameter("uavcan_node_id").as_int();
  uavcan_node_name_ = this->get_parameter("uavcan_node_name").as_string();
  update_time_ = this->get_parameter("update_time").as_double();
  max_service_call_retries_ = this->get_parameter("max_service_call_retries").as_int();
  spin_duration_ = this->get_parameter("spin_duration").as_double();

  if (update_time_ < spin_duration_) {
    throw std::runtime_error("Update time must be greater than spin duration.");
  }
}

void WiboticCanDriverNode::CreateWiboticCanDriver()
{
  wibotic_can_driver_->ConfigureUavCan(
    can_iface_name_, uavcan_node_id_, uavcan_node_name_, max_service_call_retries_);
  wibotic_can_driver_->CreateUavCanNode();
  wibotic_can_driver_->CreateWiboticInfoSubscriber();
  wibotic_can_driver_->Activate();

  wibotic_can_driver_->SetChargerRequestedState(false);
  wibotic_can_driver_->CallServiceAndSpinForResponse();
}

wibotic::WiBoticInfo WiboticCanDriverNode::GetWiboticInfo()
{
  const auto spin_duration_ms = static_cast<std::size_t>(spin_duration_ * 1000);
  wibotic_can_driver_->Spin(spin_duration_ms);

  return wibotic_can_driver_->GetWiboticInfo();
}

void WiboticCanDriverNode::WiboticInfoTimerCallback()
{
  if (!wibotic_can_driver_) {
    throw std::runtime_error("Trying to get WiboticInfo message from nonexisting driver.");
  }

  wibotic::WiBoticInfo wibotic_info;
  try {
    wibotic_info = GetWiboticInfo();
  } catch (const std::runtime_error & e) {
    // Skip if there is no messages.
    return;
  }

  try {
    wibotic_info_pub_->publish(ConvertWiboticInfoToMsg(wibotic_info));
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN(this->get_logger(), e.what());
  }
}

void WiboticCanDriverNode::WiboticChargerEnableCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::string message = "Charger correctly changed state.";
  if (!wibotic_can_driver_) {
    message = "Trying to enable charger on nonexisting driver.";
    RCLCPP_ERROR_STREAM(this->get_logger(), message);
    response->success = false;
    response->message = message;
    return;
  }

  wibotic_can_driver_->SetChargerRequestedState(request->data);

  try {
    wibotic_can_driver_->CallServiceAndSpinForResponse();
    response->success = wibotic_can_driver_->GetChargerState() == request->data;
    response->message = message;
  } catch (const std::runtime_error & e) {
    message = "Failed to change charger state: " + std::string(e.what());
    RCLCPP_ERROR_STREAM(this->get_logger(), message);
    response->success = false;
    response->message = message;
    return;
  }
}

wibotic_msgs::msg::WiboticInfo WiboticCanDriverNode::ConvertWiboticInfoToMsg(
  const wibotic::WiBoticInfo & wibotic_info)
{
  wibotic_msgs::msg::WiboticInfo wibotic_info_msg;

  wibotic_info_msg.header.stamp = this->now();
  wibotic_info_msg.header.frame_id = "wibotic_receiver";
  wibotic_info_msg.v_mon_batt = wibotic_info.VMonBatt;
  wibotic_info_msg.i_battery = wibotic_info.IBattery;
  wibotic_info_msg.v_rect = wibotic_info.VRect;
  wibotic_info_msg.v_mon_charger = wibotic_info.VMonCharger;
  wibotic_info_msg.t_board = wibotic_info.TBoard;
  wibotic_info_msg.target_i_batt = wibotic_info.TargetIBatt;
  wibotic_info_msg.i_charger = wibotic_info.ICharger;
  wibotic_info_msg.i_single_charger2 = wibotic_info.ISingleCharger2;
  wibotic_info_msg.i_single_charger3 = wibotic_info.ISingleCharger3;

  return wibotic_info_msg;
}

}  // namespace wibotic_connector_can
