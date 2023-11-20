// "Copyright [year] <Copyright Owner>"

#include "as2_core/core_functions.hpp"
#include "as2_platform_template.hpp"

using namespace as2_platform_template;

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<As2PlatformTemplate>();

  node->preset_loop_frequency(50);  // Node frequency for run and
                                    // callbacks

  // Node with only callbacks
  as2::spinLoop(node);

  // Node with run
  // as2::spinLoop(node,std::bind(&As2PlatformTemplate::run, node));

  rclcpp::shutdown();
  return 0;
}
