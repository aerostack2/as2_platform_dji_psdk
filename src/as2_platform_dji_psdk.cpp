#include "as2_platform_dji_psdk.hpp"

namespace as2_platform_dji_psdk
{

DJIMatricePSDKPlatform::DJIMatricePSDKPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options) {}

void DJIMatricePSDKPlatform::configureSensors()
{
  // Configure sensors here
}

bool DJIMatricePSDKPlatform::ownSetArmingState(bool state)
{
  // Set arming state here
  return false;
}

bool DJIMatricePSDKPlatform::ownSetOffboardControl(bool offboard)
{
  // Set offboard control here
  return false;
}

bool DJIMatricePSDKPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  // Set platform control mode here
  return false;
}

bool DJIMatricePSDKPlatform::ownSendCommand()
{
  // Send command to platform here
  return false;
}

void DJIMatricePSDKPlatform::ownStopPlatform()
{
  // Send hover to platform here
}

void DJIMatricePSDKPlatform::ownKillSwitch()
{
  // Send kill switch to platform here
}

bool DJIMatricePSDKPlatform::ownTakeoff()
{
  // Send takeoff to platform here
  return false;
}

bool DJIMatricePSDKPlatform::ownLand()
{
  // Send land to platform here
  return false;
}

}  // namespace as2_platform_dji_psdk
