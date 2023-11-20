/*!*******************************************************************************************
 *  \file       as2_platform_dji_psdk.hpp
 *  \brief      DJIMatricePSDKPlatform class header file.
 *  \authors    Rafael Pérez Seguí
 *              Santiago Tapia Fernández
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef AS2_PLATFORM_DJI_PSDK_HPP_
#define AS2_PLATFORM_DJI_PSDK_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/aerial_platform.hpp"
#include "as2_msgs/msg/control_mode.hpp"

#include "psdk_interfaces/msg/gimbal_rotation.hpp"

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

private:
  psdk_interfaces::msg::GimbalRotation gimbal_rotation_msg_;
};  // class DJIMatricePSDKPlatform

}  // namespace as2_platform_dji_psdk

#endif  // AS2_PLATFORM_DJI_PSDK_HPP_
