// Copyright (c) 2026 Franka Robotics GmbH
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

// Pinocchio header
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

namespace franka_selfcollision {
class SelfCollisionChecker {
 public:
  /**
   * @brief Constructor loads the URDF/SRDF and builds the Pinocchio models.
   * @param urdf_xml XML of the robots URDF file.
   * @param srdf_xml XML of the robots SRDF file (for disabling allowable collisions).
   * @param security_margin Safety buffer in meters (default 0.045).
   * @param logger ROS logger for reporting.
   * @param clock ROS clock for throttling logs if necessary.
   */
  SelfCollisionChecker(const std::string& urdf_xml,
                       const std::string& srdf_xml,
                       double security_margin,
                       rclcpp::Logger logger,
                       rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Checks if the given joint configuration results in a self-collision.
   * @param joint_configuration Vector of joint positions.
   * @param print_collisions If true, prints the names of colliding links to stdout.
   * @return true if collision detected, false otherwise.
   */
  bool checkCollision(const std::vector<double>& joint_configuration,
                      bool print_collisions = false);

  /**
   * @brief Eigen function for faster checking.
   * @param q Eigen vector reference containing the joint positions.
   * @param print_collisions If true, the names of colliding link pairs are printed.
   * @return true if a collision is detected, false otherwise.
   */
  bool checkCollisions(const Eigen::VectorXd& q, bool print_collisions = false);

  /**
   * @brief Retrieves the list of joint names from the loaded model.
   * @return A vector of strings containing joint names in order.
   */
  const std::vector<std::string>& getModelJointNames() const { return model_.names; }

 private:
  pinocchio::Model model_;
  pinocchio::GeometryModel geom_model_;
  std::shared_ptr<pinocchio::Data> data_;
  std::shared_ptr<pinocchio::GeometryData> geom_data_;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace franka_selfcollision