#ifndef LANELET_MISSION_PLANNER__LANELET_MISSION_PLANNER_
#define LANELET_MISSION_PLANNER__LANELET_MISSION_PLANNER_

#include "lanelet2_mission_planner/lanelet2_mission_planner_utils.hpp"

// ROS2 messages
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

class LaneletMissionPlanner : public rclcpp::Node
{
public:
	explicit LaneletMissionPlanner();

private:
	//Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

	//Publisher
	rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr global_path_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_global_path_pub_;

	// Callbacks
	void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void map_callback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

	// Variables
	nav_msgs::msg::Odometry odometry_;
	autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_ptr_;
	geometry_msgs::msg::PoseStamped goal_pose_stamped_;
	geometry_msgs::msg::Pose goal_pose_;
	geometry_msgs::msg::Pose start_pose_;
	bool ready_to_plan = false;
	bool lanelet_map_set = false;
	// TF2
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	// Lanelet
	lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;

	// Functions
	geometry_msgs::msg::PoseStamped transform_pose(const geometry_msgs::msg::PoseStamped&);
	bool is_goal_valid(const geometry_msgs::msg::Pose&);
	void setupTF();
};

#endif //LANELET_MISSION_PLANNER__LANELET_MISSION_PLANNER_HPP_