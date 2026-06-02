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

#include <iostream>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "wibotic_connector_can/wibotic_can_driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto wibotic_can_driver = std::make_shared<wibotic_connector_can::WiboticCanDriver>();
    auto wibotic_can_driver_node = std::make_shared<wibotic_connector_can::WiboticCanDriverNode>(
      "wibotic_can_driver", wibotic_can_driver);
    rclcpp::spin(wibotic_can_driver_node);
  } catch (const std::runtime_error & e) {
    std::cerr << "[wibotic_can_driver] Caught exception: " << e.what() << std::endl;
  }

  std::cout << "[wibotic_can_driver] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}
