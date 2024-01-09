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
#include "std_srvs/srv/trigger.hpp"
#include "as2_core/synchronous_service_client.hpp"

// #include "psdk_interfaces/msg/gimbal_rotation.hpp"

namespace as2_platform_dji_psdk
{

struct VelocityCommand
{
  using Msg_t                    = sensor_msgs::msg::Joy;
  inline static std::string name = "psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate";
  // TODO(stapia): Check the actual command name
};

// Service to turn motors on
struct TurnOnMotors
{
  using Msg_t                    = std_srvs::srv::Trigger;
  inline static std::string name = "psdk_ros2/turn_on_motors";
};

// Service to turn motors off
struct TurnOffMotors
{
  using Msg_t                    = std_srvs::srv::Trigger;
  inline static std::string name = "psdk_ros2/turn_off_motors";
};

// Service to takeoff
struct Takeoff
{
  using Msg_t                    = std_srvs::srv::Trigger;
  inline static std::string name = "psdk_ros2/takeoff";
};

// Service to land
struct Land
{
  using Msg_t                    = std_srvs::srv::Trigger;
  inline static std::string name = "psdk_ros2/land";
};

// Service to set initial reference should be called before any control command
// Note that this is a service used in the psdk wrapper to transform the coordiante
// frames later
struct SetLocalPositionService
{
  using Msg_t                    = std_srvs::srv::Trigger;
  inline static std::string name = "/psdk_ros2/set_local_position_ref";
};

struct ObtainCtrlAuthorityService
{
  using Msg_t                    = std_srvs::srv::Trigger;
  inline static std::string name = "psdk_ros2/obtain_ctrl_authority";
};

class DJIMatricePSDKPlatform_impl
{
public:
  Output<VelocityCommand> velocityCommand;
  as2::SynchronousServiceClient<TurnOnMotors::Msg_t> turnOnMotorsService;
  as2::SynchronousServiceClient<TurnOffMotors::Msg_t> turnOffMotorsService;
  as2::SynchronousServiceClient<Takeoff::Msg_t> takeoffService;
  as2::SynchronousServiceClient<Land::Msg_t> landService;
  as2::SynchronousServiceClient<SetLocalPositionService::Msg_t> setLocalPositionService;
  as2::SynchronousServiceClient<ObtainCtrlAuthorityService::Msg_t> obtainCtrlAuthorityService;

public:
  DJIMatricePSDKPlatform_impl(as2::Node * node_);
  void init(rclcpp::Node *);
  // psdk_interfaces::msg::GimbalRotation gimbal_rotation_msg_;
};

}  // namespace as2_platform_dji_psdk

#endif  // DETAILS__AS2_DJI_MATRICE_PSDK_PLATFORM_IMPL_HPP_
