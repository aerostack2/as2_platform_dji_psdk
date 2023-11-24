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


#ifndef DETAILS__AS2_DJI_MATRICE_PSDK_PLATFORM_IMPL_HPP_
#define DETAILS__AS2_DJI_MATRICE_PSDK_PLATFORM_IMPL_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "output.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "service_client.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "timer_with_rate.hpp"

// #include "psdk_interfaces/msg/gimbal_rotation.hpp"

namespace as2_platform_dji_psdk
{

struct VelocityCommand
{
  using Msg_t = sensor_msgs::msg::Joy;
  inline static std::string name = "/psdk_ros2/flight_control_setpoint_ENUposition_yaw";
  // TODO(stapia): Check the actual command name
};

// Service to set initial reference should be called before any control command
struct SetLocalPositionService
{
  using Msg_t = std_srvs::srv::Trigger;
  inline static std::string name = "/psdk_ros2/set_local_position_ref";
};

struct ObtainCtrlAuthorityService
{
  using Msg_t = std_srvs::srv::Trigger;
  inline static std::string name = "psdk_ros2/obtain_ctrl_authority";
};

class DJIMatricePSDKPlatform_impl : public TimerWithRate
{
public:
  Output<VelocityCommand> velocityCommand;
  ServiceClient<SetLocalPositionService> setLocalPositionService;
  ServiceClient<ObtainCtrlAuthorityService> obtainCtrlAuthorityService;

public:
  DJIMatricePSDKPlatform_impl();
  void init(rclcpp::Node *);
  virtual void timer_tick() override; // From i_TimerTick -< TimerWithRate

  // psdk_interfaces::msg::GimbalRotation gimbal_rotation_msg_;
};

}  // namespace as2_platform_dji_psdk

#endif  // DETAILS__AS2_DJI_MATRICE_PSDK_PLATFORM_IMPL_HPP_
