#ifndef MPC_PLANNER__MPC_PLANNER_HPP_
#define MPC_PLANNER__MPC_PLANNER_HPP_

#define NX     NMPC_PLANNER_NX
#define NU     NMPC_PLANNER_NU
#define NBX0   NMPC_PLANNER_NBX0

#include <rclcpp/rclcpp.hpp>

// Msgs
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_nmpc_planner.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// TF2
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <memory>
#include <cmath>
#include <array>

class MPCPlanner : public rclcpp::Node
{
public:	
	explicit MPCPlanner();
	~MPCPlanner();

private:
	//Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr global_path_sub_;

	//Publishers
	rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_local_path_pub_;

	// callbacks
	void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr);
	void global_path_callback(const autoware_planning_msgs::msg::Path::ConstSharedPtr);

	//timer
	rclcpp::TimerBase::SharedPtr timer_;
	void timer_callback();

	//tf2
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	//Acados variables
	nmpc_planner_solver_capsule *acados_ocp_capsule;
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
	autoware_planning_msgs::msg::Path path_msg_;
	double acceleration_;
	double prev_twist_ = 0.0;
	bool initialized = false;
	bool path_recieved_ = false;
	bool odometry_recieved_ = false;
	double init_timer_;
	double curr_velocity_;
	double LOOK_AHEAD_TIME = 4.0;
	int MIN_GOAL_IDX = 6;
	int radius_inf = 500;

	struct refPose {
		double x;
		double y;
		double yaw;
	};

  // Functions
  void setMPCProblem();
  void setupTF();
	std::tuple<geometry_msgs::msg::Pose, int> setGoal(autoware_planning_msgs::msg::Path&, double&, int&);
	int getCurrentIndex(std::vector<autoware_planning_msgs::msg::PathPoint>&, nav_msgs::msg::Odometry&);
	double setVelocity(const int&, const autoware_planning_msgs::msg::Path&);
	double getCurvature(std::array<int, 3>&, const autoware_planning_msgs::msg::Path&);
	size_t findNearestIndex(const std::vector<autoware_planning_msgs::msg::PathPoint>&, const geometry_msgs::msg::Point&);
	double calcSquaredDistance2d(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&);
	autoware_planning_msgs::msg::Trajectory getLocalPathFromMPC(double[], double[]);
	geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double&);
	void createLocalPathMarker(const autoware_planning_msgs::msg::Trajectory&, 
																		const geometry_msgs::msg::Pose&, visualization_msgs::msg::MarkerArray&);
	refPose transformGoalToBase(const nav_msgs::msg::Odometry&, const geometry_msgs::msg::Pose&);

};

#endif //MPC_PLANNER__MPC_PLANNER_HPP_