
#include "lanelet2_mission_planner/lanelet2_mission_planner_utils.hpp"

// ROS2 messages
#include <nav_msgs/Odometry.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <autoware_auto_mapping_msgs/msg/h_a_d_map_bin.hpp>

// TF2
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

class LaneletMissionPlanner : public rclcpp::Node
{
public:
	explicit LaneletMissionPlanner()

private:
	//Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub_;
	rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::ConstSharedPtr map_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::ConstSharedPtr goal_pose_sub_;

	//Publisher
	rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr global_path_pub_;

	// Callbacks
	void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void map_callback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
  void goal_pose_callback(const geometry_msgs::msg::ConstSharedPtr msg);

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
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	// Lanelet
	lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;

	// Functions
	geometry_msgs::msg::PoseStamped transform_pose(const PoseStamped&)
	bool is_goal_valid(const geometry_msgs::msg::Pose&)
	void setupTF();
};