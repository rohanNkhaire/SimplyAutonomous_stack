#include "nmpc_path_tracker/nmpc_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
