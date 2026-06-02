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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

// RCLCPP is compiling with C++17, so we need to define UAVCAN_CPP_VERSION to UAVCAN_CPP11
// to avoid compilation errors and silent them.
#define UAVCAN_CPP_VERSION UAVCAN_CPP11
#include "wibotic_connector_can/wibotic_can_driver.hpp"
#include "wibotic_connector_can/wibotic_can_driver_node.hpp"

namespace wibotic
{

// This is dummy stream overload because the real one is in the uavcan library what is built with
// C++11/
std::ostream & operator<<(std::ostream & os, const WiBoticInfo &) { return os; }
std::ostream & operator<<(std::ostream & os, const uavcan::protocol::param::GetSet::Request &)
{
  return os;
}

}  // namespace wibotic

class MockWiboticCanDriver : public wibotic_connector_can::WiboticCanDriverInterface
{
public:
  MOCK_METHOD(
    void, ConfigureUavCan, (const std::string &, std::size_t, const std::string &, std::size_t),
    (override));
  MOCK_METHOD(void, CreateUavCanNode, (), (override));
  MOCK_METHOD(void, CreateWiboticInfoSubscriber, (), (override));
  MOCK_METHOD(void, Activate, (), (override));
  MOCK_METHOD(void, Spin, (std::size_t), (override));
  MOCK_METHOD(wibotic::WiBoticInfo, GetWiboticInfo, (), (override));
  MOCK_METHOD(void, CallServiceAndSpinForResponse, (), (override));
  MOCK_METHOD(void, SetChargerRequestedState, (bool), ());
  MOCK_METHOD(bool, GetChargerState, (), ());

  // Nice mock suppresses warnings about uninteresting calls
  using NiceMock = testing::NiceMock<MockWiboticCanDriver>;
};

class WiboticCanDriverNodeWrapper : public wibotic_connector_can::WiboticCanDriverNode
{
public:
  WiboticCanDriverNodeWrapper(
    const std::string & node_name,
    wibotic_connector_can::WiboticCanDriverInterface::SharedPtr wibotic_can_driver,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : wibotic_connector_can::WiboticCanDriverNode(node_name, wibotic_can_driver, options)
  {
  }

  wibotic::WiBoticInfo GetWiboticInfo()
  {
    return wibotic_connector_can::WiboticCanDriverNode::GetWiboticInfo();
  }

  wibotic_msgs::msg::WiboticInfo ConvertWiboticInfoToMsg(const wibotic::WiBoticInfo & wibotic_info)
  {
    return wibotic_connector_can::WiboticCanDriverNode::ConvertWiboticInfoToMsg(wibotic_info);
  }
};

class TestWiboticCanDriverNode : public ::testing::Test
{
public:
  TestWiboticCanDriverNode();

protected:
  std::shared_ptr<MockWiboticCanDriver> wibotic_can_driver_;
  std::unique_ptr<WiboticCanDriverNodeWrapper> wibotic_can_driver_node_;
};

TestWiboticCanDriverNode::TestWiboticCanDriverNode()
{
  wibotic_can_driver_ = std::make_shared<MockWiboticCanDriver>();
  wibotic_can_driver_node_ = std::make_unique<WiboticCanDriverNodeWrapper>(
    "wibotic_can_driver", wibotic_can_driver_);
}

TEST_F(TestWiboticCanDriverNode, GetChargerState)
{
  EXPECT_FALSE(wibotic_can_driver_->GetChargerState());
}

TEST_F(TestWiboticCanDriverNode, CallServiceAndSpinForResponseFail)
{
  ON_CALL(*wibotic_can_driver_, CallServiceAndSpinForResponse())
    .WillByDefault(testing::Throw(std::runtime_error("Service call retries exceeded.")));

  EXPECT_THROW(wibotic_can_driver_->CallServiceAndSpinForResponse(), std::runtime_error);
}

TEST_F(TestWiboticCanDriverNode, GetWiboticInfoEmptyQueue)
{
  ON_CALL(*wibotic_can_driver_, GetWiboticInfo())
    .WillByDefault(testing::Throw(std::runtime_error("Queue is empty!")));

  EXPECT_THROW(wibotic_can_driver_node_->GetWiboticInfo(), std::runtime_error);
}

TEST_F(TestWiboticCanDriverNode, GetWiboticInfo)
{
  wibotic::WiBoticInfo wibotic_info;
  wibotic_info.VMonBatt = 1.0;
  ON_CALL(*wibotic_can_driver_, GetWiboticInfo()).WillByDefault([wibotic_info]() {
    return wibotic_info;
  });

  EXPECT_EQ(wibotic_can_driver_node_->GetWiboticInfo(), wibotic_info);
}

TEST_F(TestWiboticCanDriverNode, ConvertWiboticInfoToMsg)
{
  wibotic::WiBoticInfo wibotic_info;
  wibotic_info.VMonBatt = 1.0;
  wibotic_info.IBattery = 2.0;
  wibotic_info.VRect = 3.0;
  wibotic_info.VMonCharger = 4.0;
  wibotic_info.TBoard = 5.0;
  wibotic_info.TargetIBatt = 6.0;
  wibotic_info.ICharger = 7.0;
  wibotic_info.ISingleCharger2 = 8.0;
  wibotic_info.ISingleCharger3 = 9.0;

  auto wibotic_info_msg = wibotic_can_driver_node_->ConvertWiboticInfoToMsg(wibotic_info);

  EXPECT_FLOAT_EQ(wibotic_info_msg.v_mon_batt, 1.0);
  EXPECT_FLOAT_EQ(wibotic_info_msg.i_battery, 2.0);
  EXPECT_FLOAT_EQ(wibotic_info_msg.v_rect, 3.0);
  EXPECT_FLOAT_EQ(wibotic_info_msg.v_mon_charger, 4.0);
  EXPECT_FLOAT_EQ(wibotic_info_msg.t_board, 5.0);
  EXPECT_FLOAT_EQ(wibotic_info_msg.target_i_batt, 6.0);
  EXPECT_FLOAT_EQ(wibotic_info_msg.i_charger, 7.0);
  EXPECT_FLOAT_EQ(wibotic_info_msg.i_single_charger2, 8.0);
  EXPECT_FLOAT_EQ(wibotic_info_msg.i_single_charger3, 9.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
