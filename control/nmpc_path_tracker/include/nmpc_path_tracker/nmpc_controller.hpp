#ifndef NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_
#define NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

class PathTracker : public rclcpp::Node
{
public:
  explicit PathTracker();

private:
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr local_traj_sub_;

	// Variables
	nav_msgs::msg::Odometry::ConstSharedPtr odometry_ptr_;
	autoware_planning_msgs::msg::Trajectory::ConstSharedPtr local_traj_ptr_;

	//timer
	rclcpp::TimerBase::SharedPtr timer_;
	void timer_callback();

	// callbacks
	void odomcallback(const nav_msgs::msg::Odometry::ConstSharedPtr);
	void localtrajcallback(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr);

};

#endif //NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_