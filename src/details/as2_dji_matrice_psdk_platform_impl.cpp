
#include "as2_dji_matrice_psdk_platform_impl.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2_platform_dji_psdk
{

DJIMatricePSDKPlatform_impl::DJIMatricePSDKPlatform_impl() {}

void DJIMatricePSDKPlatform_impl::init(rclcpp::Node * node)
{
  velocityCommand.init(node);
  setLocalPositionService.init(node);
}

}  // namespace as2_platform_dji_psdk
