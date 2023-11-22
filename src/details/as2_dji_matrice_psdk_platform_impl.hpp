
#ifndef DETAILS__AS2_DJI_MATRICE_PSDK_PLATFORM_IMPL_HPP_
#define DETAILS__AS2_DJI_MATRICE_PSDK_PLATFORM_IMPL_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "output.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "service_client.hpp"
#include "std_srvs/srv/trigger.hpp"

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

class DJIMatricePSDKPlatform_impl
{
public:
  Output<VelocityCommand> velocityCommand;
  ServiceClient<SetLocalPositionService> setLocalPositionService;

public:
  DJIMatricePSDKPlatform_impl();
  void init(rclcpp::Node *);

  // psdk_interfaces::msg::GimbalRotation gimbal_rotation_msg_;
};

}  // namespace as2_platform_dji_psdk

#endif  // DETAILS__AS2_DJI_MATRICE_PSDK_PLATFORM_IMPL_HPP_
