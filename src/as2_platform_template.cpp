// "Copyright [year] <Copyright Owner>"

#include "as2_platform_template.hpp"

using namespace as2_platform_template;

As2PlatformTemplate::As2PlatformTemplate(const rclcpp::NodeOptions &options)
    : as2::AerialPlatform(options) {}

void As2PlatformTemplate::configureSensors() {
  // Configure sensors here
}

bool As2PlatformTemplate::ownSetArmingState(bool state) {
  // Set arming state here
  return false;
}

bool As2PlatformTemplate::ownSetOffboardControl(bool offboard) {
  // Set offboard control here
  return false;
}

bool As2PlatformTemplate::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg) {
  // Set platform control mode here
  return false;
}

bool As2PlatformTemplate::ownSendCommand() {
  // Send command to platform here
  return false;
}

void As2PlatformTemplate::ownStopPlatform() {
  // Send hover to platform here
}

void As2PlatformTemplate::ownKillSwitch() {
  // Send kill switch to platform here
}

bool As2PlatformTemplate::ownTakeoff() {
  // Send takeoff to platform here
  return false;
}

bool As2PlatformTemplate::ownLand() {
  // Send land to platform here
  return false;
}