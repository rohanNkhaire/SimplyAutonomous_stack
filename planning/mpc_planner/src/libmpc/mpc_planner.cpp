#include "mpc_planner/mpc_planner.hpp"

// Declare plugin
pluginlib::ClassLoader<libmpc::LibMPCBase> nmpc_loader("mpc_plugin", "libmpc::LibMPCBase");
std::shared_ptr<libmpc::LibMPCBase> planner = nmpc_loader.createSharedInstance("nmpc_planner::NMPCPlanner");

MPCPlanner::MPCPlanner() : Node("mpc_planner")
{
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", rclcpp::QoS{1}, std::bind(&MPCPlanner::odom_callback, this, std::placeholders::_1));

	global_path_sub_ = create_subscription<autoware_planning_msgs::msg::Path>("/lanelet_mission_planner/path", rclcpp::QoS{1},
																									std::bind(&MPCPlanner::global_path_callback, this, std::placeholders::_1));

  trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("/mpc_planner/traj", rclcpp::QoS{10});
	viz_local_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visual_local_path", rclcpp::QoS{1});


	// Timer
	timer_ = create_wall_timer(std::chrono::milliseconds(80), std::bind(&MPCPlanner::timer_callback, this));

	setupTF();
	planner->initialize();
																																			
}

void MPCPlanner::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	odometry_ = *msg;
	odometry_recieved_ = true;							
}

void MPCPlanner::global_path_callback(const autoware_planning_msgs::msg::Path::ConstSharedPtr msg)
{
	path_msg_ = *msg;
	RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "NMPC planner recieved global path");
	path_recieved_ = true;
}

void MPCPlanner::timer_callback()
{
	bool transform_available = tf_buffer_->canTransform("base_link", "map", rclcpp::Time(), tf2::Duration(std::chrono::milliseconds(80)));

	if(path_recieved_ && transform_available)
	{
		// Get current index on the global path
		int curr_index = getCurrentIndex(path_msg_.points, odometry_);

		// Set the goal point and goal index in the global path
		std::tuple<geometry_msgs::msg::Pose, int> goal_tuple = setGoal(path_msg_, odometry_.twist.twist.linear.x, curr_index);

		// Set desired velocity
		double ref_vel = setVelocity(std::get<1>(goal_tuple), path_msg_);

		// Tranform goal pose w.r.t base_link
    refPose goal_st = transformGoalToBase(odometry_, std::get<0>(goal_tuple));

		// Set the reference for MPC
		Eigen::Vector4d yref;
		yref(0) = goal_st.x;
		yref(1) = goal_st.y;
		yref(2) = goal_st.yaw;
		yref(3) = ref_vel;

		// Set the trajectory
		Eigen::Matrix<double, 21, 3> traj_mat;
		for(int i = 0; i < 21; ++i)
		{
			traj_mat(i, 0) = goal_st.x;
			traj_mat(i, 1) = goal_st.y;
			traj_mat(i, 2) = goal_st.yaw;
		}
		
		int end_idx = std::min((std::get<1>(goal_tuple) - curr_index), int(traj_mat.rows()));
		for(int i = 0; i < end_idx; ++i)
		{
			int start_idx = curr_index + i;
      refPose inter_goal_st = transformGoalToBase(odometry_, path_msg_.points.at(start_idx).pose);
			traj_mat(i, 0) = inter_goal_st.x;
			traj_mat(i, 1) = inter_goal_st.y;
			traj_mat(i, 2) = inter_goal_st.yaw;
		}

		planner->setTrajectory(traj_mat);
		// Set reference for controller
		planner->setReference(yref);

		Eigen::Vector4d modelX;
		modelX(0) = 0.0;
		modelX(1) = 0.0;
		modelX(2) = 0.0;
		modelX(3) = odometry_.twist.twist.linear.x;

		// Set the input for controller
		planner->setStateInput(modelX);

		RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "NMPC planner generating trajectory");		
		// Calc Optimal control
		planner->stepController();

		// Get optimal states and inputs
		std::unique_ptr<Eigen::MatrixXd> opt_states_ptr(new Eigen::MatrixXd(20, 4));
		std::unique_ptr<Eigen::MatrixXd> opt_inputs_ptr(new Eigen::MatrixXd(20, 2));
		planner->getOptimalStates(opt_states_ptr);
		planner->getOptimalInputs(opt_inputs_ptr);

		RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "PRINTING OPTIMAL CONTROL INPUTS");	
		Eigen::MatrixXd opt_inputs = *opt_inputs_ptr;
		Eigen::MatrixXd opt_states = *opt_states_ptr;

		autoware_planning_msgs::msg::Trajectory local_path = getLocalPathFromMPC(opt_states, opt_inputs);

		// Publish the trajectory
		trajectory_pub_->publish(local_path);

		// Publish visualization
		visualization_msgs::msg::MarkerArray visual_local_path;
    createLocalPathMarker(local_path, std::get<0>(goal_tuple), visual_local_path);
		viz_local_path_pub_->publish(visual_local_path);

	}
}

std::tuple<geometry_msgs::msg::Pose, int> MPCPlanner::setGoal(autoware_planning_msgs::msg::Path& path, double& curr_vel, int& curr_idx)
{
	// Set goal such that we plan for 4s minimum
	// We take goal from the global planner
	// scale goal with vehicle speed
	int max_length = path.points.size() - 1;
	int min_goal_idx = curr_idx + MIN_GOAL_IDX;
	int scaled_vel_idx = curr_idx + int(std::round(curr_vel*LOOK_AHEAD_TIME));

	// Choose proper goal idx
	int intermediate_goal_idx = std::max(scaled_vel_idx, min_goal_idx); 
	int goal_idx = std::min(intermediate_goal_idx, max_length);

	geometry_msgs::msg::Pose goal_pose = path.points.at(goal_idx).pose;

	return std::make_tuple(goal_pose, goal_idx);
}

int MPCPlanner::getCurrentIndex(std::vector<autoware_planning_msgs::msg::PathPoint>& path_point, nav_msgs::msg::Odometry& veh_odom)
{
	int curr_index = findNearestIndex(path_point, veh_odom.pose.pose.position);
	return curr_index;
}

double MPCPlanner::setVelocity(const int& goal_idx, const autoware_planning_msgs::msg::Path& path)
{
	// If end of path
	int end_idx = path.points.size() - 1;
	if (goal_idx == end_idx)
	{
		return 0.0;
	}

	if (goal_idx == 0)
	{
		return 5.0;
	}

	// calculate suitable velocity based on points before and after goal_idx
	std::array<int, 3> curve_points;
	curve_points[0] = goal_idx - 1;
	curve_points[1] = goal_idx;
	curve_points[2] = goal_idx + 1;

	// Get curvature based on prev and ahead points
	double curvature = getCurvature(curve_points, path);

	// Set reference velocity according to the curvature
	double ref_vel = (curvature < radius_inf) ? sqrt(1.0 * curvature) : 5.0;

	return ref_vel;

}

double MPCPlanner::getCurvature(std::array<int, 3>& pt_idx, const autoware_planning_msgs::msg::Path& path)
{
	const double d = 2 * ((path.points.at(pt_idx[0]).pose.position.y - path.points.at(pt_idx[2]).pose.position.y) * 
												(path.points.at(pt_idx[0]).pose.position.x - path.points.at(pt_idx[1]).pose.position.x) - 
												(path.points.at(pt_idx[0]).pose.position.y - path.points.at(pt_idx[1]).pose.position.y) * 
												(path.points.at(pt_idx[0]).pose.position.x - path.points.at(pt_idx[2]).pose.position.x));

	if (fabs(d) < 1e-8)
	{
		return radius_inf;
	}											

  const std::array<double, 3> x2 = { path.points.at(pt_idx[0]).pose.position.x * path.points.at(pt_idx[0]).pose.position.x, 
																			path.points.at(pt_idx[1]).pose.position.x * path.points.at(pt_idx[1]).pose.position.x, 
																			path.points.at(pt_idx[2]).pose.position.x * path.points.at(pt_idx[2]).pose.position.x };
  const std::array<double, 3> y2 = { path.points.at(pt_idx[0]).pose.position.y * path.points.at(pt_idx[0]).pose.position.y, 
																			path.points.at(pt_idx[1]).pose.position.y * path.points.at(pt_idx[1]).pose.position.y, 
																			path.points.at(pt_idx[2]).pose.position.y * path.points.at(pt_idx[2]).pose.position.y };

  const double a = y2[0] - y2[1] + x2[0] - x2[1];
  const double b = y2[0] - y2[2] + x2[0] - x2[2];

  const double cx = ((path.points.at(pt_idx[0]).pose.position.y - path.points.at(pt_idx[2]).pose.position.y) * a - 
																(path.points.at(pt_idx[0]).pose.position.y - path.points.at(pt_idx[1]).pose.position.y) * b) / d;
  const double cy = ((path.points.at(pt_idx[0]).pose.position.x - path.points.at(pt_idx[2]).pose.position.x) * a - 
																(path.points.at(pt_idx[0]).pose.position.x - path.points.at(pt_idx[1]).pose.position.x) * b) / -d;

  double curv = sqrt((cx - path.points.at(pt_idx[0]).pose.position.x) * (cx - path.points.at(pt_idx[0]).pose.position.x) + 
										 (cy - path.points.at(pt_idx[0]).pose.position.y) * (cy - path.points.at(pt_idx[0]).pose.position.y));
  return curv;
}

autoware_planning_msgs::msg::Trajectory MPCPlanner::getLocalPathFromMPC(const Eigen::MatrixXd& states, const Eigen::MatrixXd& inputs)
{
	autoware_planning_msgs::msg::Trajectory local_traj;
	auto opt_states = states;
	auto opt_input = inputs;
	int horizon_length = states.rows();
	for (int i = 0; i < horizon_length; ++i)
	{
		autoware_planning_msgs::msg::TrajectoryPoint traj_point;
		geometry_msgs::msg::Pose traj_point_pose;
		traj_point_pose.position.x = opt_states(i, 0);
		traj_point_pose.position.y = opt_states(i, 1);
		traj_point_pose.position.z = 0.0;
		auto traj_point_quat = createQuaternionFromYaw(opt_states(i, 2));
		traj_point_pose.orientation.w = traj_point_quat.w;
		traj_point_pose.orientation.x = traj_point_quat.x;
		traj_point_pose.orientation.y = traj_point_quat.y;
		traj_point_pose.orientation.z = traj_point_quat.z;
		
		traj_point.pose = traj_point_pose;
		traj_point.longitudinal_velocity_mps = opt_states(i, 3);
		traj_point.acceleration_mps2 = opt_input(i, 0);
		traj_point.heading_rate_rps = opt_input(i, 1);

		RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "NMPC planner opt states- u0:%f, u1:%f", opt_input(i, 0), opt_input(i, 1));
		
		
		local_traj.points.push_back(traj_point);
	}

	return local_traj;
}

size_t MPCPlanner::findNearestIndex(const std::vector<autoware_planning_msgs::msg::PathPoint>& points, const geometry_msgs::msg::Point& point)
{

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = calcSquaredDistance2d(points.at(i).pose.position, point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

double MPCPlanner::calcSquaredDistance2d(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2)
{
  const auto p1 = point1;
  const auto p2 = point2;
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;
  return dx * dx + dy * dy;
}

geometry_msgs::msg::Quaternion MPCPlanner::createQuaternionFromYaw(const double& yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void MPCPlanner::createLocalPathMarker(const autoware_planning_msgs::msg::Trajectory &lane_waypoints_array, 
																		const geometry_msgs::msg::Pose& goal_pose, visualization_msgs::msg::MarkerArray& markerArray)
{
  std_msgs::msg::ColorRGBA total_color;
  total_color.r = 1.0;
  total_color.g = 0.2;
  total_color.b = 0.3;
  total_color.a = 1.0;
   
  visualization_msgs::msg::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "base_link";
  lane_waypoint_marker.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  lane_waypoint_marker.ns = "local_trajectory_marker";
  lane_waypoint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
  lane_waypoint_marker.scale.x = 1.0;
  lane_waypoint_marker.scale.y = 1.0;
  lane_waypoint_marker.color = total_color;
  lane_waypoint_marker.frame_locked = true;

  int count = 0;
  for (unsigned int i=0; i<lane_waypoints_array.points.size(); i++)
  {
    geometry_msgs::msg::Point point = lane_waypoints_array.points.at(i).pose.position;
    lane_waypoint_marker.points.push_back(point);

    count++;
  }

	markerArray.markers.push_back(lane_waypoint_marker);

	// Goal Point
	visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  goal_marker.ns = "goal_marker";
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.scale.x = 1.0;
  goal_marker.scale.y = 1.0;
	goal_marker.scale.z = 1.0;
  goal_marker.color = total_color;
  goal_marker.frame_locked = true;
	goal_marker.pose = goal_pose;

	markerArray.markers.push_back(goal_marker);

}

MPCPlanner::refPose MPCPlanner::transformGoalToBase(const nav_msgs::msg::Odometry& veh_odom, const geometry_msgs::msg::Pose& goal_pose)
{
  refPose goal_struct;
  geometry_msgs::msg::PoseStamped pose_in, pose_out;
  pose_in.header = veh_odom.header;
  pose_in.pose = goal_pose;

  
  tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(pose_in, pose_out, "base_link",
        tf2::Duration(std::chrono::milliseconds(50)));

  goal_struct.x = pose_out.pose.position.x;
  goal_struct.y = pose_out.pose.position.y;
  goal_struct.yaw = tf2::getYaw(pose_out.pose.orientation); 

  return goal_struct; 
}

geometry_msgs::msg::Point MPCPlanner::transformBaseToMap(const geometry_msgs::msg::Pose& goal_pose)
{
  geometry_msgs::msg::Point out_point;
  geometry_msgs::msg::PoseStamped pose_in, pose_out;
  pose_in.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
	pose_in.header.frame_id = "base_link";
  pose_in.pose = goal_pose;

  tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(pose_in, pose_out, "map",
        tf2::Duration(std::chrono::milliseconds(50)));

	out_point = pose_out.pose.position;

  return out_point; 
}

void MPCPlanner::setupTF()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}