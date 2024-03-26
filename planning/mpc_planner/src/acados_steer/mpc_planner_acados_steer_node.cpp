#include "mpc_planner/mpc_planner_acados_steer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPCPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
