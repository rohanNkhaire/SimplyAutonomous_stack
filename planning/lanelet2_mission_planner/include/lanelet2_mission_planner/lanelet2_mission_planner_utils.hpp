#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

// Importing lanelet2 headers
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>

#include <vector>
#include <memory>

class LaneletMissionPlannerUtils : public rclcpp::Node
{
public:
	explicit LaneletMissionPlannerUtils()

private:
	geometry_msgs::msg::Pose refine_goal_pose(geometry_msgs::msg::Pose&);
	void plan();
	void collect_lanelets();
	void
}