#include "lanelet2_mission_planner/lanelet2_mission_planner.hpp"

void LaneletMissionPlanner::setupTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

LaneletMissionPlanner::LaneletMissionPlanner() : Node("lanelet2_mission_planner")
{
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", rclcpp::QoS(1),
    std::bind(&LaneletMissionPlanner::odom_callback, this, std::placeholders::_1));
  const auto durable_qos = rclcpp::QoS(1).transient_local();
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>("/map/vector_map", durable_qos,
    std::bind(&LaneletMissionPlanner::map_callback, this, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(1),
    std::bind(&LaneletMissionPlanner::goal_pose_callback, this, std::placeholders::_1));  

  global_path_pub_ = create_publisher<autoware_planning_msgs::msg::Path>("/lanelet_mission_planner/path", rclcpp::QoS{10});  
  viz_global_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visual_global_path", rclcpp::QoS{1});

  setupTF();

}

geometry_msgs::msg::PoseStamped LaneletMissionPlanner::transform_pose(const geometry_msgs::msg::PoseStamped& input)
{
  geometry_msgs::msg::PoseStamped output;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", input.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input, output, transform);
    return output;
  } catch (tf2::TransformException & error) {
    throw std::invalid_argument(error.what());
  }
}

bool LaneletMissionPlanner::is_goal_valid(const geometry_msgs::msg::Pose& goal)
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
    if (LaneletMissionPlannerUtils::is_in_lane(closest_road_lanelet, goal_lanelet_pt)) {
      const auto lane_yaw = lanelet::utils::getLaneletAngle(closest_road_lanelet, goal.position);
      // Get Goal Yaw
      tf2::Quaternion goal_quat_;
      tf2::Matrix3x3 m;
      tf2::fromMsg(goal.orientation, goal_quat_);
      m.setRotation(goal_quat_);
      double goal_roll, goal_pitch, goal_yaw;
      m.getRPY(goal_roll, goal_pitch, goal_yaw);

      const auto angle_diff = LaneletMissionPlannerUtils::normalizeRadian(lane_yaw - goal_yaw);
      const double th_angle = LaneletMissionPlannerUtils::deg2rad(45);
      if (std::abs(angle_diff) < th_angle) {
        return true;
      }
    }
  }

  return false;      
}

void LaneletMissionPlanner::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odometry_ = *msg;

  // Plan when vehicle is at standstill
  if(odometry_.twist.twist.linear.x == 0.0)
  {
    ready_to_plan = true;
  }
}

void LaneletMissionPlanner::map_callback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  map_ptr_ = msg;

  // Creating lanelet map
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet_map_set = true;
}

void LaneletMissionPlanner::goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{

  goal_pose_stamped_ = transform_pose(*msg);
  goal_pose_ = goal_pose_stamped_.pose;
  start_pose_ = odometry_.pose.pose;
  // Check validity of the goal
  if(!is_goal_valid(goal_pose_))
  {
    RCLCPP_WARN(get_logger(), "The Goal is invalid");
    return;
  }

  if(ready_to_plan && lanelet_map_set)
  {
    // Make the goal on the centreline
    geometry_msgs::msg::Pose refined_goal = LaneletMissionPlannerUtils::get_closest_centerline_pose(road_lanelets_, goal_pose_);
    // Get start and goal lanelet
    lanelet::Lanelet start_lanelet, goal_lanelet;
    lanelet::utils::query::getClosestLaneletWithConstrains(road_lanelets_, start_pose_, &start_lanelet, 0.0);
    lanelet::utils::query::getClosestLaneletWithConstrains(road_lanelets_, refined_goal, &goal_lanelet, 0.0);

    // Use route graph to collect all lanelets
    lanelet::Optional<lanelet::routing::LaneletPath> shortest_path_opt = routing_graph_ptr_->shortestPath(start_lanelet, goal_lanelet);
    lanelet::routing::LaneletPath shortest_path;

    if (!shortest_path_opt)
    {
      RCLCPP_WARN(get_logger(), "Could not find path in lanelet map!");
      return;
    }

    shortest_path = shortest_path_opt.value();
    lanelet::LaneletSequence continuous_lane = shortest_path.getRemainingLane(shortest_path.begin());

    if (continuous_lane.size() != shortest_path.size())
    {
      RCLCPP_WARN(get_logger(), "This route contains a lane change which is currently unsupported");
      return;
    }

    RCLCPP_INFO(get_logger(), "Found a path containing %lu lanelets", shortest_path.size());

    // Store as Path
    autoware_planning_msgs::msg::Path path_msg = LaneletMissionPlannerUtils::generate_path_points(continuous_lane, start_lanelet, goal_lanelet,
                                                                                      start_pose_.position, goal_pose_.position);
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

    visualization_msgs::msg::MarkerArray visual_global_path;
    LaneletMissionPlannerUtils::createGlobalLaneArrayMarker(path_msg, visual_global_path);

    // Publish and visualize path
    global_path_pub_->publish(path_msg);
    viz_global_path_pub_->publish(visual_global_path);
  }
}