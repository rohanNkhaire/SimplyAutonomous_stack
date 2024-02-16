#include "lanelet2_mission_planner/lanelet2_mission_planner_utils.hpp"

LaneletMissionPlannerUtils::LaneletMissionPlannerUtils()
{
}

double project_goal_to_map(const lanelet::Lanelet & lanelet_component, const lanelet::ConstPoint3d & goal_point)
{
  const lanelet::ConstLineString3d center_line =
    lanelet::utils::generateFineCenterline(lanelet_component);
  lanelet::BasicPoint3d project = lanelet::geometry::project(center_line, goal_point.basicPoint());
  return project.z();
}

geometry_msgs::msg::Pose get_closest_centerline_pose(
  const lanelet::ConstLanelets & road_lanelets, const geometry_msgs::msg::Pose & point)
{
  lanelet::Lanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLaneletWithConstrains(
        road_lanelets, point, &closest_lanelet, 0.0)) {
    // point is not on any lanelet.
    return point;
  }

  const auto refined_center_line = lanelet::utils::generateFineCenterline(closest_lanelet, 1.0);
  closest_lanelet.setCenterline(refined_center_line);

  const double lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, point.position);

  const auto nearest_idx =
    motion_utils::findNearestIndex(convertCenterlineToPoints(closest_lanelet), point.position);
  const auto nearest_point = closest_lanelet.centerline()[nearest_idx];

  return convertBasicPoint3dToPose(nearest_point, lane_yaw);
}

geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d & point, const double lane_yaw)
{
  // calculate new orientation of goal
  geometry_msgs::msg::Pose pose;
  pose.position.x = point.x();
  pose.position.y = point.y();
  pose.position.z = point.z();

  pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(lane_yaw);

  return pose;
}
