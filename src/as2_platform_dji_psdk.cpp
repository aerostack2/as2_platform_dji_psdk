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
