#include "lanelet2_mission_planner/lanelet2_mission_planner_utils.hpp"

namespace LaneletMissionPlannerUtils
{

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

  std::vector<geometry_msgs::msg::Point> centerline_points = convertCenterlineToPoints(closest_lanelet);
  const auto nearest_idx = findNearestIndex(centerline_points, point.position);
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

  pose.orientation = createQuaternionFromYaw(lane_yaw);

  return pose;
}

std::vector<geometry_msgs::msg::Point> convertCenterlineToPoints(lanelet::Lanelet & lanelet)
{
  std::vector<geometry_msgs::msg::Point> centerline_points;
  for (const auto & point : lanelet.centerline()) {
    geometry_msgs::msg::Point center_point;
    center_point.x = point.basicPoint().x();
    center_point.y = point.basicPoint().y();
    center_point.z = point.basicPoint().z();
    centerline_points.push_back(center_point);
  }
  return centerline_points;
}

autoware_planning_msgs::msg::Path generate_path_points(
  const lanelet::LaneletSequence& continuous_lane, const geometry_msgs::msg::Pose& goal_point)
{
  autoware_planning_msgs::msg::Path path;

  // Loop over each lanelet
  for (auto& lanelet : continuous_lane.lanelets())
  {

    const auto refined_center_line = lanelet::utils::generateFineCenterline(lanelet, 1.0);
    const int wp_length = refined_center_line.size() - 1;

    // Loop over each centerline point
    for (int i = 0; i <= wp_length; i++)
    {
      autoware_planning_msgs::msg::PathPoint waypoints;

      auto point = refined_center_line[i];
      const lanelet::BasicPoint3d basic_center_point = point.basicPoint();

      geometry_msgs::msg::Point center_point;
      center_point.x = basic_center_point.x();
      center_point.y = basic_center_point.y();
      center_point.z = basic_center_point.z();

      const double lane_yaw = lanelet::utils::getLaneletAngle(lanelet, center_point);
      geometry_msgs::msg::Pose new_wp = convertBasicPoint3dToPose(basic_center_point, lane_yaw);

      waypoints.longitudinal_velocity_mps = 5.0;
      waypoints.pose = new_wp;

      path.points.push_back(waypoints);
    }
  }

  return path;
}

bool is_in_lane(const lanelet::ConstLanelet & lanelet, const lanelet::ConstPoint3d & point)
{
  // check if goal is on a lane at appropriate angle
  const auto distance = boost::geometry::distance(
    lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(point).basicPoint());
  constexpr double th_distance = std::numeric_limits<double>::epsilon();
  return distance < th_distance;
}

geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

size_t findNearestIndex(std::vector<geometry_msgs::msg::Point>& points, const geometry_msgs::msg::Point& point)
{

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = calcSquaredDistance2d(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

double calcSquaredDistance2d(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2)
{
  const auto p1 = point1;
  const auto p2 = point2;
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;
  return dx * dx + dy * dy;
}

void createGlobalLaneArrayMarker(const autoware_planning_msgs::msg::Path &lane_waypoints_array, visualization_msgs::msg::MarkerArray& markerArray)
{
  std_msgs::msg::ColorRGBA total_color;
  total_color.r = 0.5;
  total_color.g = 0.7;
  total_color.b = 1.0;
  total_color.a = 0.9;
   
  visualization_msgs::msg::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  lane_waypoint_marker.ns = "global_lane_array_marker";
  lane_waypoint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
  lane_waypoint_marker.scale.x = 1.0;
  lane_waypoint_marker.scale.y = 1.0;
  lane_waypoint_marker.color = total_color;
  lane_waypoint_marker.frame_locked = true;

  int count = 0;
  for (unsigned int i=0; i<lane_waypoints_array.points.size(); i++)
  {
    geometry_msgs::msg::Point point;
    point = lane_waypoints_array.points.at(i).pose.position;
    lane_waypoint_marker.points.push_back(point);

    count++;
  }
  
  markerArray.markers.push_back(lane_waypoint_marker);
}

} // namespace LaneletMissionPlannerUtils