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

#include "wibotic_connector_can/wibotic_can_driver.hpp"

namespace wibotic_connector_can
{
void WiboticCanDriver::ConfigureUavCan(
  const std::string & can_iface_name, std::size_t node_id, const std::string & node_name,
  std::size_t max_service_call_retries)
{
  can_iface_name_ = can_iface_name;
  max_service_call_retries_ = max_service_call_retries;
  node_id_ = node_id;
  node_name_ = node_name;
}

void WiboticCanDriver::CreateUavCanNode()
{
  uavcan_node_ = uavcan_linux::makeNode({can_iface_name_});
  uavcan_node_->setNodeID(node_id_);
  uavcan_node_->setName(node_name_.c_str());
}

void WiboticCanDriver::CreateWiboticInfoSubscriber()
{
  wibotic_info_uavcan_sub_ =
    std::make_shared<uavcan::Subscriber<wibotic::WiBoticInfo>>(*uavcan_node_);
  uavcan_param_client_ =
    std::make_shared<uavcan::ServiceClient<uavcan::protocol::param::GetSet>>(*uavcan_node_);
}

void WiboticCanDriver::Activate()
{
  if (!uavcan_node_) {
    throw std::runtime_error("Trying to activate nonexisting node.");
  }

  const int sub_res = wibotic_info_uavcan_sub_->start(
    WiBoticInfoCallbackBinder(this, &WiboticCanDriver::WiboticInfoCallback));
  if (sub_res < 0) {
    throw std::runtime_error("Failed to start the subscriber; error: " + std::to_string(sub_res));
  }

  const int client_init_res = uavcan_param_client_->init();
  if (client_init_res < 0) {
    throw std::runtime_error(
      "Failed to init the client; error: " + std::to_string(client_init_res));
  }

  uavcan_param_client_->setRequestTimeout(uavcan::MonotonicDuration::fromMSec(100));
  uavcan_param_client_->setCallback(ParamCallbackBinder(this, &WiboticCanDriver::ParamCallback));

  uavcan_node_->setModeOperational();

  const int node_start_res = uavcan_node_->start();
  if (node_start_res < 0) {
    throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
  }

  activated_ = true;
}

void WiboticCanDriver::Spin(std::size_t milliseconds)
{
  if (!uavcan_node_) {
    throw std::runtime_error("Trying to spin nonexisting node.");
  }

  if (!wibotic_info_uavcan_sub_) {
    throw std::runtime_error("Trying to spin nonexisting subscriber.");
  }

  if (!activated_) {
    throw std::runtime_error("Trying to spin non-activated driver.");
  }

  const int res = uavcan_node_->spin(uavcan::MonotonicDuration::fromMSec(milliseconds));
  if (res < 0) {
    throw std::runtime_error("Failed to spin UAVCAN node, res: " + std::to_string(res));
  }
}

wibotic::WiBoticInfo WiboticCanDriver::GetWiboticInfo()
{
  if (wibotic_info_queue_.empty()) {
    throw std::runtime_error("WiBoticInfo queue is empty.");
  }

  wibotic::WiBoticInfo wibotic_info = wibotic_info_queue_.front();
  wibotic_info_queue_.pop();
  return wibotic_info;
}

void WiboticCanDriver::WiboticInfoCallback(const wibotic::WiBoticInfo & msg)
{
  wibotic_info_queue_.push(msg);
}

void WiboticCanDriver::CallServiceAndSpinForResponse()
{
  if (charger_enabled_requested_state_ == charger_enabled_actual_state_) {
    return;
  }

  auto request = uavcan::protocol::param::GetSet::Request();
  request.index = WIBOTIC_CHARGER_ENABLE_PARAM_INDEX;
  request.value.to<uavcan::protocol::param::Value::Tag::integer_value>() =
    charger_enabled_requested_state_;

  std::uint8_t service_call_retries = 0;

  CallParamService(request);

  while (charger_enabled_requested_state_ != charger_enabled_actual_state_) {
    try {
      Spin(100);
    } catch (const std::runtime_error & e) {
      std::cerr << "WiboticCan Driver: " << e.what()
                << " Trial number: " << (int)service_call_retries + 1 << "/"
                << (int)max_service_call_retries_ << std::endl;

      if (service_call_retries == max_service_call_retries_ - 1u) {
        throw std::runtime_error("Service call retries exceeded.");
      }

      CallParamService(request);
      service_call_retries++;
    }
  }
}

void WiboticCanDriver::CallParamService(uavcan::protocol::param::GetSet::Request & request)
{
  const int call_res = uavcan_param_client_->call(WIBOTIC_RECEIVER_NODE_ID, request);
  if (call_res < 0) {
    throw std::runtime_error("Unable to perform service call: " + std::to_string(call_res));
  }
}

void WiboticCanDriver::ParamCallback(
  const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> & result)
{
  if (result.isSuccessful()) {
    std::cout << "WiboticCan Driver: Enable service call successful" << std::endl;

    auto value = result.getResponse().value.integer_value;
    charger_enabled_actual_state_ = static_cast<bool>(value);
  } else {
    throw std::runtime_error("Service call to node has failed.");
  }
}

}  // namespace wibotic_connector_can
