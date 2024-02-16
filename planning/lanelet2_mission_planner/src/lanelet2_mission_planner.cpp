#include "lanelet2_mission_planner/lanelet2_mission_planner.hpp"

LaneletMissionPlanner::LaneletMissionPlanner()
{
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry("/localization/kinematic_state", rclcpp::QoS(1),
    std::bind(&LaneletMissionPlanner::odom_callback, std::placeholders::_1));
  const auto durable_qos = rclcpp::QoS(1).transient_local();
  sub_vector_map_ = create_subscription<HADMapBin>("input/vector_map", durable_qos,
    std::bind(&LaneletMissionPlanner::map_callback, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(1),
    std::bind(&LaneletMissionPlanner::goal_pose_callback, std::placeholders::_1));  

}

geometry_msgs::msg::PoseStamped LaneletMissionPlanner::transform_pose(const geometry_msgs::msg::PoseStamped& input)
{
  geometry_msgs::msg::PoseStamped output;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", input.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input, output, transform);
    return output;
  } catch (tf2::TransformException & error) {
    throw component_interface_utils::TransformError(error.what());
  }
}

bool is_goal_valid(const geometry_msgs::msg::Pose& goal)
{
  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal.position);

  lanelet::Lanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets_, goal, &closest_lanelet)) {
    return false;
  }

  // check if goal is in road lanelet
  lanelet::Lanelet closest_road_lanelet;
  if (lanelet::utils::query::getClosestLanelet(
        road_lanelets_, goal, &closest_road_lanelet)) {
    if (is_in_lane(closest_road_lanelet, goal_lanelet_pt)) {
      const auto lane_yaw =
        lanelet::utils::getLaneletAngle(closest_shoulder_lanelet, goal.position);
      const auto goal_yaw = tf2::getYaw(goal.orientation);
      const auto angle_diff = tier4_autoware_utils::normalizeRadian(lane_yaw - goal_yaw);
      const double th_angle = tier4_autoware_utils::deg2rad(45);
      if (std::abs(angle_diff) < th_angle) {
        return true;
      }
    }
  }

  return false      
}

void LaneletMissionPlanner::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odometry_ = &msg;

  // Plan when vehicle is at standstill
  if(odometry_.twist.twist.linear.x == 0.0)
  {
    ready_to_plan = true;
  }
}

void LaneletMissionPlanner::map_callback(const HADMapBin::ConstSharedPtr msg)
{
  map_ptr_ = msg;

  // Creating lanelet map
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);
  lanelet_map_set = true;
}

void LaneletMissionPlanner::goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{

  goal_pose_ = transform_pose(msg->pose);

  // Check validity of the goal
  is_goal_valid(goal_pose_);

  // Make the goal on the centreline
  geometry_msgs::msg::Pose refined_goal = LaneletMissionPlannerUtils::refine_goal_pose(goal_pose);

  if(ready_to_plan && lanelet_map_set)
  {
    // Plan
    // Collect all lanel
  }
}