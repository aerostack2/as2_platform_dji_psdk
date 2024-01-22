// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "as2_platform_dji_psdk.hpp"
#include "details/as2_dji_matrice_psdk_platform_impl.hpp"

namespace as2_platform_dji_psdk
{

DJIMatricePSDKPlatform::DJIMatricePSDKPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing DJI Matrice PSDK platform");
  _impl = std::make_shared<DJIMatricePSDKPlatform_impl>(this);
  configureSensors();
}

void DJIMatricePSDKPlatform::configureSensors() { 
  RCLCPP_INFO(this->get_logger(), "Configuring sensors");
  _impl->init(this); }

bool DJIMatricePSDKPlatform::ownSetArmingState(bool state)
{
  // TODO: (stapia) ¿Fijar la posición va antes o después de arrancar motores?
  // Set Local Position at the begining of flight
  auto request  = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  bool sr       = _impl->setLocalPositionService.sendRequest(request, response);
  bool success  = sr && response->success;
  if (!success) {
    RCLCPP_INFO(
      this->get_logger(), "Send request was not succeed due to '%s'", response->message.data());
    return success;
  }
  // // Turn on motors
  // sr      = _impl->turnOnMotorsService.sendRequest(request, response);
  // success = sr && response->success;
  // if (!success) {
  //   RCLCPP_INFO(
  //     this->get_logger(), "Send request was not succeed due to '%s'", response->message.data());
  //   return success;
  // }
  return success;
}

bool DJIMatricePSDKPlatform::ownSetOffboardControl(bool offboard)
{
  // // Turn on motors
  // auto request  = std::make_shared<std_srvs::srv::Trigger::Request>();
  // auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  // bool sr       = _impl->turnOffMotorsService.sendRequest(request, response);
  // bool success  = sr && response->success;
  // if (!success) {
  //   RCLCPP_INFO(
  //     this->get_logger(), "Send request was not succeed due to '%s'", response->message.data());
  //   return success;
  // }
  // return success;
  return true;
}

bool DJIMatricePSDKPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
   RCLCPP_INFO(
      this->get_logger(), "Setting control mode to %d", msg.control_mode);
  // Obtain control authority
  // TODO: (stapia) ¿Dónde hay que soltar la autorización?
  auto request  = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  bool sr       = _impl->obtainCtrlAuthorityService.sendRequest(request, response);
  bool success  = sr && response->success;
  if (!success) {
    RCLCPP_INFO(
      this->get_logger(), "Send request was not succeed due to '%s'", response->message.data());
    return success;
  }
  // Note: Since velocity command in psdk wrapper already has the configuration of mode and
  // command sending there is no need to store anything else.
  return success;
}

bool DJIMatricePSDKPlatform::ownSendCommand()
{
  if (platform_info_msg_.current_control_mode.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    // send all zeros
    _impl->velocityCommand.msg().axes[0] = 0.0f;  // x(m)
    _impl->velocityCommand.msg().axes[1] = 0.0f;
    _impl->velocityCommand.msg().axes[2] = 0.0f;
    _impl->velocityCommand.msg().axes[3] = 0.0f;  // yaw (rad)
    _impl->velocityCommand.publish();
    return true;
  }
  // Check whether is a new input for joystick command
  if (!this->has_new_references_) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "No new references since mode change");
    return true;
  }

  switch (platform_info_msg_.current_control_mode.control_mode) {
    // case as2_msgs::msg::ControlMode::POSITION: {
    // } break;
    case as2_msgs::msg::ControlMode::SPEED:
      {
        // Conversion from AS2 ENU frame into DJI NEU frame already done inside psdk wrapper
        _impl->velocityCommand.msg().axes[0] = this->command_twist_msg_.twist.linear.x;
        _impl->velocityCommand.msg().axes[1] = this->command_twist_msg_.twist.linear.y;
        _impl->velocityCommand.msg().axes[2] = this->command_twist_msg_.twist.linear.z;
        _impl->velocityCommand.msg().axes[3] = this->command_twist_msg_.twist.angular.z;
        _impl->velocityCommand.publish();
      }
      break;
    // case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE: {
    // } break;
    // case as2_msgs::msg::ControlMode::ACRO: {
    // } break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown control mode in send command");
      return false;
  }
  return true;
}

void DJIMatricePSDKPlatform::ownStopPlatform()
{
  platform_info_msg_.current_control_mode.control_mode = as2_msgs::msg::ControlMode::HOVER;
}

void DJIMatricePSDKPlatform::ownKillSwitch()
{
  // Send kill switch to platform here
  // TODO: (stapia) ¿Qué hay que hacer aquí?
}

bool DJIMatricePSDKPlatform::ownTakeoff()
{
  auto request  = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  RCLCPP_INFO(this->get_logger(), "Takeoff request sent");
  bool sr       = _impl->takeoffService.sendRequest(request, response);
  bool success  = sr && response->success;
  // TODO: (stapia) Meter delay para esperar a takeoff (10s)
  if (!success) {
    RCLCPP_INFO(
      this->get_logger(), "Send request was not succeed due to '%s'", response->message.data());
  }
  return success;
}

bool DJIMatricePSDKPlatform::ownLand()
{
  auto request  = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  bool sr       = _impl->landService.sendRequest(request, response);
  bool success  = sr && response->success;
  if (!success) {
    RCLCPP_INFO(
      this->get_logger(), "Send request was not succeed due to '%s'", response->message.data());
  }
  return success;
}

}  // namespace as2_platform_dji_psdk
