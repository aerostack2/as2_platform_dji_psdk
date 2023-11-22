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


#ifndef AS2_PLATFORM_DJI_PSDK__AS2_PLATFORM_DJI_PSDK_HPP_
#define AS2_PLATFORM_DJI_PSDK__AS2_PLATFORM_DJI_PSDK_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "as2_core/aerial_platform.hpp"
#include "as2_msgs/msg/control_mode.hpp"

namespace as2_platform_dji_psdk
{

class DJIMatricePSDKPlatform_impl;

class DJIMatricePSDKPlatform : public as2::AerialPlatform
{
public:
  explicit DJIMatricePSDKPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~DJIMatricePSDKPlatform() = default;

public:
  void configureSensors() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  bool ownSendCommand() override;
  void ownStopPlatform() override;
  void ownKillSwitch() override;
  bool ownTakeoff() override;
  bool ownLand() override;

private:
  std::shared_ptr<DJIMatricePSDKPlatform_impl> _impl;
};  // class DJIMatricePSDKPlatform

}  // namespace as2_platform_dji_psdk

#endif  // AS2_PLATFORM_DJI_PSDK__AS2_PLATFORM_DJI_PSDK_HPP_
