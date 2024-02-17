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
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <vector>
#include <memory>
#include <cmath>

class LaneletMissionPlannerUtils
{
public:
	explicit LaneletMissionPlannerUtils()

	constexpr double pi = 3.14159265358979323846;

	inline double normalizeRadian(const double rad, const double min_rad = -pi)
	{
	  const auto max_rad = min_rad + 2 * pi;

	  const auto value = std::fmod(rad, 2 * pi);
	  if (min_rad <= value && value < max_rad) {
	    return value;
	  }

	  return value - std::copysign(2 * pi, value);
	}

	constexpr double deg2rad(const double deg)
	{
	  return deg * pi / 180.0;
	}
	constexpr double rad2deg(const double rad)
	{
	  return rad * 180.0 / pi;
	}

	double project_goal_to_map(const lanelet::Lanelet &, const lanelet::ConstPoint3d &);
	geometry_msgs::msg::Pose get_closest_centerline_pose(
  const lanelet::ConstLanelets &, const geometry_msgs::msg::Pose &);

  geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d &, const double);

  std::vector<geometry_msgs::msg::Point> convertCenterlineToPoints(const lanelet::Lanelet&);
  autoware_planning_msgs::msg::PathPoint Ll2GlobalPlannerNl::generate_path_points(
  const LaneletSequence&, const geometry_msgs::msg::Pose&);

  bool is_in_lane(const lanelet::ConstLanelet &, const lanelet::ConstPoint3d &);
  geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double);
  size_t findNearestIndex(std::vector<geometry_msgs::msg::Point>&, const geometry_msgs::msg::Point&);
  double calcSquaredDistance2d(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&);
}