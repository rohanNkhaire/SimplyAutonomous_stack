#include "lanelet2_mission_planner/lanelet2_mission_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneletMissionPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
