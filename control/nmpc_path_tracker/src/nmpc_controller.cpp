#include "nmpc_path_tracker/nmpc_controller.hpp"

PathTracker::PathTracker() : Node("nmpc_path_tracker")
{
	// Subcription
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("localisation/kinematic_state", rclcpp::QoS{5},
																														std::bind(&PathTracker::odomcallback, this, std::placeholders::_1));

	local_traj_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>("mpc_planner/traj", rclcpp::QoS{1},
																														std::bind(&PathTracker::localtrajcallback, this, std::placeholders::_1));

	//Publish


	// Timer
	timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&PathTracker::timer_callback, this));
}

void PathTracker::odomcallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	odometry_ptr_ = msg;
}

void PathTracker::localtrajcallback(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
	local_traj_ptr_ = msg;
}

void timer_callback()
{

}