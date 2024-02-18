#include "mpc_planner/mpc_planner.hpp"

struct mpc_parameters
{
	constexpr int num_states = 2;
  constexpr int num_output = 2;
  constexpr int num_inputs = 1;
  constexpr int pred_hor = 10;
  constexpr int ctrl_hor = 5;
  constexpr int ineq_c = pred_hor + 1;
  constexpr int eq_c = 0;
  double ts = 0.1;
};


MPCPlanner::MPCPlanner() : Node("mpc_planner")
{
	odom_sub_ = create_subscription<nav_msgs::msg:Odometry>("/localization/kinematic_state", rclcpp::QoS{10},
																				std::bind(&MPCPlanner::odom_callback, this, std::placeholders::_1));
	const auto durable_qos = rclcpp::QoS(1).transient_local();
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>("/map/vector_map", durable_qos,
																								std::bind(&MPCPLanner::map_callback, this, std::placeholders::_1));

  trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("/mpc_planner/traj", rclcpp::QoS{10});
																																												
}

void MPCPlanner::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
	odometry_ = *msg;

	// wait for 3 seconds at the start
	if (!initialized)
	{
		double curr_timer = get_clock()->now().seconds();
		init_timer_ = get_clock()->now().seconds();
		while (init_timer_ - curr_timer > 3.0)
		{
			prev_odometry_ = *msg;
		}
		initialized = true;
	}

	acceleration_ = (odometry_.twist.twist.linear.x - prev_odometry_.twist.twist.linear.x)/
									(odometry_.header.stamp.sec - prev_odometry_.header.stamp.sec);

	prev_odometry_ = odometry_;								
}

void MPCPlanner::map_callback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  map_ptr_ = msg;

  // Creating lanelet map
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
}

void MPCPLanner::setMPCProblem()
{

}