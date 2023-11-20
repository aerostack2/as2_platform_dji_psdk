// "Copyright [year] <Copyright Owner>"

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "as2_platform_template/as2_platform_template.hpp"

using namespace as2_platform_template;

std::shared_ptr<As2PlatformTemplate> get_node() {
  const std::string package_path =
      ament_index_cpp::get_package_share_directory("as2_platform_template");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  rclcpp::NodeOptions node_options;
    fi
  node_options.parameter_overrides(
      {{"control_modes_file", rclcpp::ParameterValue(control_modes_config_file)}});
  return std::make_shared<As2PlatformTemplate>(node_options);
}

TEST(As2PlatformTemplate, test_constructor) {
  EXPECT_NO_THROW(std::shared_ptr<As2PlatformTemplate> node = get_node());
}

TEST(As2PlatformTemplate, test_virtual_methods) {
  std::shared_ptr<As2PlatformTemplate> node = get_node();
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