#ifndef MPC_PLANNER__MPC_PLANNER_HPP_
#define MPC_PLANNER__MPC_PLANNER_HPP_

#include <mpc/LMPC.hpp>
#include <mpc/NLMPC.hpp> 
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

// MPC
	constexpr int num_states = 4;
  constexpr int num_output = 2;
  constexpr int num_inputs = 2;
  constexpr int pred_hor = 10;
  constexpr int ctrl_hor = 10;
  constexpr int ineq_c = 0;
  constexpr int eq_c = 0;
  double ts = 0.02;

	mpc::NLMPC<
      num_states, num_inputs, num_output,
      pred_hor, ctrl_hor,
      ineq_c, eq_c> controller;

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
	void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr&);
	void map_callback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr&);

	//timer
	void timer_callback();

	//variables
	nav_msgs::msg::Odometry odometry_;
	nav_msgs::msg::Odometry prev_odometry_;
	autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_ptr_;
	double acceleration_;
	double prev_twist_ = 0.0;
	bool initialized = false;
	double init_timer_;



	// Lanelet
	lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;

  // Functions
  void setMPCProblem();

	
};

#endif //MPC_PLANNER__MPC_PLANNER_HPP_