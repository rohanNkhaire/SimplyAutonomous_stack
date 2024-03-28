#ifndef MPC_PLANNER__MPC_PLANNER_HPP_
#define MPC_PLANNER__MPC_PLANNER_HPP_

#define NU     NMPC_KINEMATIC_CONTROLLER_NU
#define NX     NMPC_KINEMATIC_CONTROLLER_NX
#define NBX0   NMPC_KINEMATIC_CONTROLLER_NBX0

#include <rclcpp/rclcpp.hpp>

// Msgs
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_nmpc_kinematic_controller.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// TF2
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <memory>
#include <cmath>
#include <array>

class PathTracker : public rclcpp::Node
{
public:	
	explicit PathTracker();
	~PathTracker();

private:
	//Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr local_path_sub_;

	//Publishers
	rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_local_path_pub_;
	rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_pub_;

	// callbacks
	void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr);
	void local_path_callback(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr);

	//timer
	rclcpp::TimerBase::SharedPtr timer_;
	void timer_callback();

	//Acados variables
	nmpc_kinematic_controller_solver_capsule *acados_ocp_capsule;
	double* new_time_steps = nullptr;
	int status, N;
	ocp_nlp_config *nlp_config;
	ocp_nlp_dims *nlp_dims;
	ocp_nlp_in *nlp_in;
	ocp_nlp_out *nlp_out;
	ocp_nlp_solver *nlp_solver;
	void *nlp_opts;
	double lbx0[NBX0];
  double ubx0[NBX0];

	//variables
	nav_msgs::msg::Odometry odometry_;
	nav_msgs::msg::Odometry prev_odometry_;
	autoware_planning_msgs::msg::Trajectory path_msg_;
	double acceleration_;
	double prev_twist_ = 0.0;
	bool initialized = false;
	bool path_recieved_ = false;
	bool odometry_recieved_ = false;
	double init_timer_;
	double curr_velocity_;
	double* vel = nullptr;

  // Functions
  void setMPCProblem();
	autoware_control_msgs::msg::Longitudinal setVelocity(const autoware_planning_msgs::msg::Trajectory&);
	autoware_control_msgs::msg::Lateral setSteering(double[]);
	autoware_planning_msgs::msg::Trajectory getLocalPathFromMPC(double[], double[]);
	geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double&);
	void createLocalPathMarker(const autoware_planning_msgs::msg::Trajectory&, 
																		visualization_msgs::msg::MarkerArray&);

};

#endif //MPC_PLANNER__MPC_PLANNER_HPP_