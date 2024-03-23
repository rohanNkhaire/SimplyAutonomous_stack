#include "mpc_planner/mpc_planner_acados.hpp"

MPCPlanner::MPCPlanner() : Node("mpc_planner")
{
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", rclcpp::QoS{1}, std::bind(&MPCPlanner::odom_callback, this, std::placeholders::_1));

	global_path_sub_ = create_subscription<autoware_planning_msgs::msg::Path>("/lanelet_mission_planner/path", rclcpp::QoS{1},
																									std::bind(&MPCPlanner::global_path_callback, this, std::placeholders::_1));

	const auto durable_qos = rclcpp::QoS(1).transient_local();
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>("/map/vector_map", durable_qos,
																								std::bind(&MPCPlanner::map_callback, this, std::placeholders::_1));

  trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("/mpc_planner/traj", rclcpp::QoS{10});
	viz_local_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visual_local_path", rclcpp::QoS{1});


	// Timer
	timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&MPCPlanner::timer_callback, this));

  setupTF();
	setMPCProblem();
																																			
}

MPCPlanner::~MPCPlanner()
{
	// free solver
    status = nmpc_planner_acados_free(acados_ocp_capsule);
    if (status) {
        printf("nmpc_planner_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = nmpc_planner_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("nmpc_planner_acados_free_capsule() returned status %d. \n", status);
    }
}

void MPCPlanner::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	odometry_ = *msg;
	odometry_recieved_ = true;						
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
	RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "NMPC planner recieved global path");
	path_recieved_ = true;
}

void MPCPlanner::timer_callback()
{
  bool transform_available = tf_buffer_->canTransform("base_link", "map", rclcpp::Time(),tf2::Duration(std::chrono::milliseconds(20)));

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
		double yref[5];
		yref[0] = goal_st.x;
		yref[1] = goal_st.y;
		yref[2] = ref_vel;
		yref[3] = goal_st.yaw;
    yref[4] = 0.0;

		// initialization for state values
    double x_init[5];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = odometry_.twist.twist.linear.x;
    x_init[3] = 0.0;
    x_init[4] = 0.0;
    
    // initial value for control input
    double u0[2];
    u0[0] = 0.0;
    u0[1] = 0.0;

    // prepare evaluation
    double min_time = 1e12;
    double elapsed_time;

    double xtraj[205];
    double utraj[80];

    // solve ocp in loop

		RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "START POINT: x:%f, y:%f, v:%f, th:%f", x_init[0], x_init[1], x_init[2], x_init[3]);
		RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "GOAL POINT: x:%f, y:%f, v:%f, th:%f", yref[0], yref[1], yref[2], yref[3]);
		// initial condition
		ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_init);
  	ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_init);

		for(int i = 0; i < N; ++i)
		{
      ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
		}
    
    for(int i = 0; i < (std::get<1>(goal_tuple) - curr_index); ++i)
		{
      double y_intermediate[5];
      int start_idx = curr_index + i;
      refPose inter_goal_st = transformGoalToBase(odometry_, path_msg_.points.at(start_idx).pose);

      // setting intermediate traj
      y_intermediate[0] = inter_goal_st.x;
      y_intermediate[1] = inter_goal_st.y;
      y_intermediate[2] = ref_vel;
      y_intermediate[3] = inter_goal_st.yaw;
      y_intermediate[4] = 0.0;

      ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", y_intermediate);
		}
  
		for (int i = 0; i < N; i++)
    {
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }

    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
    status = nmpc_planner_acados_solve(acados_ocp_capsule);
    ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
    min_time = MIN(elapsed_time, min_time);

		for (int ii = 0; ii <= nlp_dims->N; ii++)
		{
      ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
		}
    for (int ii = 0; ii < nlp_dims->N; ii++)
		{
      ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);
		}

		if (status == ACADOS_SUCCESS)
    {
			RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "nmpc_planner_acados_solve(): SUCCESS!");
    }
    else
    {
			RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "nmpc_planner_acados_solve() failed with status %d.\n", status);
    }

		autoware_planning_msgs::msg::Trajectory local_path = getLocalPathFromMPC(xtraj, utraj);

		// Publish the trajectory
		trajectory_pub_->publish(local_path);

		// Publish visualization
		visualization_msgs::msg::MarkerArray visual_local_path;
    createLocalPathMarker(local_path, std::get<0>(goal_tuple), visual_local_path);
		viz_local_path_pub_->publish(visual_local_path);

	}
}

void MPCPlanner::setMPCProblem()
{
  acados_ocp_capsule = nmpc_planner_acados_create_capsule();
  // there is an opportunity to change the number of shooting intervals in C without new code generation
  N = NMPC_PLANNER_N;
  // allocate the array and fill it accordingly
  status = nmpc_planner_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);
  if (status)
  {
      printf("nmpc_planner_acados_create() returned status %d. Exiting.\n", status);
      exit(1);
  }

	nlp_config = nmpc_planner_acados_get_nlp_config(acados_ocp_capsule);
	nlp_dims = nmpc_planner_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = nmpc_planner_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = nmpc_planner_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = nmpc_planner_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = nmpc_planner_acados_get_nlp_opts(acados_ocp_capsule);
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

autoware_planning_msgs::msg::Trajectory MPCPlanner::getLocalPathFromMPC(double states[], double inputs[])
{
	autoware_planning_msgs::msg::Trajectory local_traj;
	int horizon_length = N;
	for (int i = 0; i < horizon_length + 1; ++i)
	{
		autoware_planning_msgs::msg::TrajectoryPoint traj_point;
		geometry_msgs::msg::Pose traj_point_pose;
		traj_point_pose.position.x = states[i*NX];
		traj_point_pose.position.y = states[(i*NX)+1];
		traj_point_pose.position.z = 0.0;
		auto traj_point_quat = createQuaternionFromYaw(states[(i*NX)+3]);
		traj_point_pose.orientation.w = traj_point_quat.w;
		traj_point_pose.orientation.x = traj_point_quat.x;
		traj_point_pose.orientation.y = traj_point_quat.y;
		traj_point_pose.orientation.z = traj_point_quat.z;
		
		traj_point.pose = traj_point_pose;
		traj_point.longitudinal_velocity_mps = states[(i*NX)+2];
    if(i < horizon_length)
    {
	  	traj_point.acceleration_mps2 = inputs[(i*NX)];
		  traj_point.heading_rate_rps = inputs[(i*NX)+1];
      RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "OPT TRAJ: u0:%f, u1:%f", inputs[(i*NX)], inputs[(i*NX)+1]);
    }

		//RCLCPP_INFO(rclcpp::get_logger("mpc_planner"), "OPT TRAJ: x:%f, y:%f, v:%f, th:%f", states[(i*NX)], states[(i*NX)+1], states[(i*NX)+2], states[(i*NX)+3]);
		

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
  lane_waypoint_marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
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

    geometry_msgs::msg::Point point;
    point = lane_waypoints_array.points.at(i).pose.position;
    lane_waypoint_marker.points.push_back(point);

    count++;
  }

	markerArray.markers.push_back(lane_waypoint_marker);

	// Goal Point
	visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
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
        tf2::Duration(std::chrono::milliseconds(10)));

  goal_struct.x = pose_out.pose.position.x;
  goal_struct.y = pose_out.pose.position.y;
  goal_struct.yaw = tf2::getYaw(pose_out.pose.orientation); 

  return goal_struct; 
}

void MPCPlanner::setupTF()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/*
geometry_msgs::msg::Point MPCPlanner::transformBaseToGoal(const geometry_msgs::msg::Pose& veh_pose, const autoware_planning_msgs::msg::Trajectory& local_traj)
{
  geometry_msgs::msg::Point traj_point;

  traj_point.x = veh_pose.position.x + 

  return goal_struct; 
}
*/