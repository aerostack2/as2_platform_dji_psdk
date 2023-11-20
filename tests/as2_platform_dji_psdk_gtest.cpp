// "Copyright [year] <Copyright Owner>"

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "as2_platform_dji_psdk/as2_platform_dji_psdk.hpp"

using namespace as2_platform_dji_psdk;

std::shared_ptr<DJIMatricePSDKPlatform> get_node() {
  const std::string package_path =
      ament_index_cpp::get_package_share_directory("as2_platform_dji_psdk");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
      {{"control_modes_file", rclcpp::ParameterValue(control_modes_config_file)}});
  return std::make_shared<DJIMatricePSDKPlatform>(node_options);
}

TEST(DJIMatricePSDKPlatform, test_constructor) {
  EXPECT_NO_THROW(std::shared_ptr<DJIMatricePSDKPlatform> node = get_node());
}

TEST(DJIMatricePSDKPlatform, test_virtual_methods) {
  std::shared_ptr<DJIMatricePSDKPlatform> node = get_node();
  EXPECT_NO_THROW(node->configureSensors());
  EXPECT_NO_THROW(node->ownSetArmingState(true));
  EXPECT_NO_THROW(node->ownSetOffboardControl(true));
  as2_msgs::msg::ControlMode msg;
  EXPECT_NO_THROW(node->ownSetPlatformControlMode(msg));
  EXPECT_NO_THROW(node->ownSendCommand());
  EXPECT_NO_THROW(node->ownStopPlatform());
  EXPECT_NO_THROW(node->ownKillSwitch());
  EXPECT_NO_THROW(node->ownTakeoff());
  EXPECT_NO_THROW(node->ownLand());
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}