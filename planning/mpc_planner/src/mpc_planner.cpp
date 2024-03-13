#include "mpc_planner/mpc_planner.hpp"

MPCPlanner::MPCPlanner() : Node("mpc_planner")
{
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/loca", rclcpp::QoS{1}, std::bind(&MPCPlanner::odom_callback, this, std::placeholders::_1));

	global_path_sub_ = create_subscription<autoware_planning_msgs::msg::Path>("/lanelet_mission_planner/path", rclcpp::QoS{1},
																									std::bind(&MPCPlanner::global_path_callback, this, std::placeholders::_1));

	const auto durable_qos = rclcpp::QoS(1).transient_local();
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>("/map/vector_map", durable_qos,
																								std::bind(&MPCPlanner::map_callback, this, std::placeholders::_1));

  trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("/mpc_planner/traj", rclcpp::QoS{10});

	// Timer
	auto timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&MPCPlanner::timer_callback, this));

	// Declare plugin
	//pluginlib::ClassLoader<libmpc::LibMPCBase> nmpc_loader("mpc_base", "libmpc::LibMPCBase");
	//std::shared_ptr<libmpc::LibMPCBase> planner = nmpc_loader.createSharedInstance("nmpc_planner::NMPCPlanner");
	//planner->initialize();
																																			
}

void MPCPlanner::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
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

void MPCPlanner::global_path_callback(const autoware_planning_msgs::msg::Path::ConstSharedPtr msg)
{
	path_msg_ = *msg;
	path_recieved_ = true;
}

void MPCPlanner::timer_callback()
{
	if(path_recieved_)
	{
		// Get current index on the global path
		int curr_index = getCurrentIndex(path_msg_.points, odometry_);

		// Set the goal point and goal index in the global path
		std::tuple<geometry_msgs::msg::Pose, int> goal_tuple = setGoal(path_msg_, odometry_.twist.twist.linear.x, curr_index);

		// Set desired velocity
		double ref_vel = setVelocity(std::get<1>(goal_tuple), path_msg_);

		// Get Goal Yaw
    tf2::Quaternion goal_quat_;
    tf2::Matrix3x3 m;
    tf2::fromMsg(std::get<0>(goal_tuple).orientation, goal_quat_);
    m.setRotation(goal_quat_);
    double goal_roll, goal_pitch, goal_yaw;
    m.getRPY(goal_roll, goal_pitch, goal_yaw);

		// Set the reference for MPC
		Eigen::Vector4d yref;
		yref(0) = std::get<0>(goal_tuple).position.x;
		yref(1) = std::get<0>(goal_tuple).position.y;
		yref(3) = goal_yaw;
		yref(4) = ref_vel;

		// Set reference for controller
		//planner->setReference(yref);

		// Get Vehicle Yaw
    tf2::Quaternion veh_quat_;
    tf2::Matrix3x3 n;
    tf2::fromMsg(odometry_.pose.pose.orientation, veh_quat_);
    n.setRotation(veh_quat_);
    double veh_roll, veh_pitch, veh_yaw;
    n.getRPY(veh_roll, veh_pitch, veh_yaw);

		Eigen::Vector4d modelX;
		modelX(0) = odometry_.pose.pose.position.x;
		modelX(1) = odometry_.pose.pose.position.y;
		modelX(2) = odometry_.twist.twist.linear.x;
		modelX(3) = veh_yaw;

		// Set the input for controller
		//planner->setStateInput(modelX);

		// Calc Optimal control
		//planner->stepController();
		
		//auto opt_states = planner->getOptimalStates();
		//auto opt_inputs = planner->getOptimalInputs();

		//autoware_planning_msgs::msg::Trajectory local_path = getLocalPathFromMPC(opt_states, opt_inputs);

		// Publish the trajectory
		//trajectory_pub_->publish(local_path);

		// Publish visualization
	

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
	int horizon_length = states.cols();
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