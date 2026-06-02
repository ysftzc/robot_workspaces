#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "franka_selfcollision/self_collision_checker.hpp"

std::string readFileToString(const std::string& filename) {
  auto file = std::ifstream(filename);
  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return "";
  }

  auto oss = std::ostringstream();
  oss << file.rdbuf();
  file.close();

  return oss.str();
}

class SelfCollisionCheckerTest : public ::testing::Test {
 protected:
  static constexpr double kSecurityMargin = 0.001;
  static constexpr size_t kNumJoints = 14;

  void SetUp() override {
    try {
      std::string test_dir = TEST_DIR;

      std::string urdf_path = test_dir + "/fr3_duo.urdf";
      std::string urdf_xml = readFileToString(urdf_path);

      std::string srdf_path = test_dir + "/fr3_duo.srdf";
      std::string srdf_xml = readFileToString(srdf_path);

      auto clock = std::make_shared<rclcpp::Clock>();

      checker_ = std::make_unique<franka_selfcollision::SelfCollisionChecker>(
          urdf_xml, srdf_xml, kSecurityMargin, rclcpp::get_logger("test_logger"), clock);

    } catch (const std::exception& e) {
      FAIL() << "Setup failed: " << e.what();
    }
  }

  std::unique_ptr<franka_selfcollision::SelfCollisionChecker> checker_;
};

TEST_F(SelfCollisionCheckerTest, givenInvalidInputDimensions_thenThrowInvalidArgument) {
  std::vector<double> input_too_small(SelfCollisionCheckerTest::kNumJoints - 1, 0.0);
  std::vector<double> input_too_big(SelfCollisionCheckerTest::kNumJoints + 1, 0.0);

  ASSERT_THROW({ checker_->checkCollision(input_too_small, false); }, std::invalid_argument);

  ASSERT_THROW({ checker_->checkCollision(input_too_big, false); }, std::invalid_argument);
}

TEST_F(SelfCollisionCheckerTest, givenSafeConfiguration_thenReturnFalse) {
  std::vector<double> home_config(SelfCollisionCheckerTest::kNumJoints, 0.0);

  // Start Configuration
  std::vector<double> start_config = {
      0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4,  // Arm 1
      0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4   // Arm 2
  };

  ASSERT_FALSE(checker_->checkCollision(home_config, true)) << "Home config should be safe";
  ASSERT_FALSE(checker_->checkCollision(start_config, true)) << "Start config should be safe";
}

TEST_F(SelfCollisionCheckerTest, givenCollidingConfiguration_thenReturnTrue) {
  // Left arm in bottom plate
  std::vector<double> mount_collision = {
      0.0, M_PI_2,  0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4,  // Arm 1
      0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4   // Arm 2
  };

  // Arms into another Configuration
  std::vector<double> dual_collision = {
      0.0, 0.2, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4,  // Arm 1
      0.0, 0.2, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4   // Arm 2
  };

  ASSERT_TRUE(checker_->checkCollision(mount_collision, true))
      << "Left arm should collide into the mount";
  ASSERT_TRUE(checker_->checkCollision(dual_collision, true))
      << "Arms should collide with each other";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}