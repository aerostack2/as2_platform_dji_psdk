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
  _impl = std::make_shared<DJIMatricePSDKPlatform_impl>(this);
}

void DJIMatricePSDKPlatform::configureSensors() {_impl->init(this);}

bool DJIMatricePSDKPlatform::ownSetArmingState(bool state)
{
  // Set Local Position at the begining of flight
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  bool sr = _impl->setLocalPositionService.sendRequest(request, response);
  bool success = sr && response->success;
  if (!success) {
    RCLCPP_INFO(
      this->get_logger(), "Send request was not succeed due to '%s'", response->message.data());
  }
  return success;
}

bool DJIMatricePSDKPlatform::ownSetOffboardControl(bool offboard)
{
  // Set offboard control here
  return false;
}

bool DJIMatricePSDKPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  // Set platform control mode here
  return true;
}

bool DJIMatricePSDKPlatform::ownSendCommand()
{
  if (platform_info_msg_.current_control_mode.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    // send all zeros
    _impl->velocityCommand->axes[0] = 0.0f;  // x(m)
    _impl->velocityCommand->axes[1] = 0.0f;
    _impl->velocityCommand->axes[2] = 0.0f;
    _impl->velocityCommand->axes[3] = 0.0f;  // yaw (rad)
    _impl->velocityCommand.publish();
    RCLCPP_INFO(this->get_logger(), " HOVERING");
    return true;
  }
  /*
  if (!this->has_new_references_) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "No new references since mode change");
    return true;
  }
  double x, y, z, yaw;
  x = y = z = yaw = 0.0;

  if (platform_info_msg_.current_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
    tf2::Quaternion q(
        this->command_pose_msg_.pose.orientation.x, this->command_pose_msg_.pose.orientation.y,
        this->command_pose_msg_.pose.orientation.z, this->command_pose_msg_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double _a, _b, yaw_;
    m.getRPY(_a, _b, yaw);
    yaw = -yaw * 180.0 / M_PI;
  } else if (platform_info_msg_.current_control_mode.yaw_mode ==
             as2_msgs::msg::ControlMode::YAW_SPEED) {
    yaw = -this->command_twist_msg_.twist.angular.z * 180.0 / M_PI;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown yaw mode");
    return false;
  }

  switch (platform_info_msg_.current_control_mode.control_mode) {
    // case as2_msgs::msg::ControlMode::POSITION: {
    // } break;
    case as2_msgs::msg::ControlMode::SPEED: {
      // Conversion from AS2 ENU frame into DJI NEU frame
      x = this->command_twist_msg_.twist.linear.y;
      y = this->command_twist_msg_.twist.linear.x;
      z = this->command_twist_msg_.twist.linear.z;
    } break;
    // case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE: {
    // } break;
    // case as2_msgs::msg::ControlMode::ACRO: {
    // } break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown control mode in send command");
      return false;
  }
  // vehicle_->flightController->setJoystickMode(dji_joystick_mode_);
  // FlightController::JoystickCommand joystick_cmd = {
  //     (float)x,
  //     (float)y,
  //     (float)z,
  //     (float)yaw,
  // };
  // vehicle_->flightController->setJoystickCommand(joystick_cmd);
  // vehicle_->flightController->joystickAction();
  */
  return true;
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
