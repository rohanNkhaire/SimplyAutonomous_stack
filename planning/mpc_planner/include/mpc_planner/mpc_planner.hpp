#ifndef MPC_PLANNER__MPC_PLANNER_HPP_
#define MPC_PLANNER__MPC_PLANNER_HPP_

#include "libmpc_plugin/libmpc_base.hpp"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Plugin lib
#include <pluginlib/class_loader.hpp>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

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

#include <memory>
#include <cmath>
#include <array>

class MPCPlanner : public rclcpp::Node
{
public:	
	explicit MPCPlanner();

private:
	//Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
	rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr global_path_sub_;

	//Publishers
	rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
	

	// callbacks
	void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr);
	void map_callback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr);
	void global_path_callback(const autoware_planning_msgs::msg::Path::ConstSharedPtr);

	//timer
	void timer_callback();

	//variables
	nav_msgs::msg::Odometry odometry_;
	nav_msgs::msg::Odometry prev_odometry_;
	autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_ptr_;
	autoware_planning_msgs::msg::Path path_msg_;
	double acceleration_;
	double prev_twist_ = 0.0;
	bool initialized = false;
	bool path_recieved_ = false;
	double init_timer_;
	double curr_velocity_;
	double LOOK_AHEAD_TIME = 4.0;
	int MIN_GOAL_IDX = 10;
	int radius_inf = 500;

	// Lanelet
	lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;

  // Functions
	std::tuple<geometry_msgs::msg::Pose, int> setGoal(autoware_planning_msgs::msg::Path&, double&, int&);
	int getCurrentIndex(std::vector<autoware_planning_msgs::msg::PathPoint>&, nav_msgs::msg::Odometry&);
	double setVelocity(const int&, const autoware_planning_msgs::msg::Path&);
	double getCurvature(std::array<int, 3>&, const autoware_planning_msgs::msg::Path&);
  void setMPCProblem();
	size_t findNearestIndex(const std::vector<autoware_planning_msgs::msg::PathPoint>&, const geometry_msgs::msg::Point&);
	double calcSquaredDistance2d(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&);
	autoware_planning_msgs::msg::Trajectory getLocalPathFromMPC(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
	geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double&);

};

#endif //MPC_PLANNER__MPC_PLANNER_HPP_