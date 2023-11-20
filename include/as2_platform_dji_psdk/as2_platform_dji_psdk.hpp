// "Copyright [year] <Copyright Owner>"

#ifndef AS2_PLATFORM_DJI_PSDK_HPP_
#define AS2_PLATFORM_DJI_PSDK_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/aerial_platform.hpp"
#include "as2_msgs/msg/control_mode.hpp"

namespace as2_platform_dji_psdk {

class DJIMatricePSDKPlatform : public as2::AerialPlatform {
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  DJIMatricePSDKPlatform(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~DJIMatricePSDKPlatform() = default;

public:
  void configureSensors() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg) override;
  bool ownSendCommand() override;
  void ownStopPlatform() override;
  void ownKillSwitch() override;
  bool ownTakeoff() override;
  bool ownLand() override;
};  // class DJIMatricePSDKPlatform

}  // namespace as2_platform_dji_psdk

#endif  // AS2_PLATFORM_DJI_PSDK_HPP_
