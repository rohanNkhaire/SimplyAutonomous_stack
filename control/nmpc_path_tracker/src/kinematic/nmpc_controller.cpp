#include "nmpc_path_tracker/nmpc_controller.hpp"

PathTracker::PathTracker() : Node("path_tracker")
{
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", rclcpp::QoS{1}, std::bind(&PathTracker::odom_callback, this, std::placeholders::_1));

	local_path_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>("/mpc_planner/traj", rclcpp::QoS{1},
																									std::bind(&PathTracker::local_path_callback, this, std::placeholders::_1));

  trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("/nmpc_controller/traj", rclcpp::QoS{10});
	viz_local_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visual_controller_path", rclcpp::QoS{1});
  control_cmd_pub_ = create_publisher<autoware_control_msgs::msg::Control>("/nmpc_controller/control_cmd", rclcpp::QoS{10});

	// Timer
	timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&PathTracker::timer_callback, this));

  // Set MPC problem
	setMPCProblem();
																																			
}

PathTracker::~PathTracker()
{
	// free solver
    status = nmpc_kinematic_controller_acados_free(acados_ocp_capsule);
    if (status) {
        printf("nmpc_planner_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = nmpc_kinematic_controller_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("nmpc_planner_acados_free_capsule() returned status %d. \n", status);
    }

    delete vel;
}

void PathTracker::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	odometry_ = *msg;
	odometry_recieved_ = true;						
}

void PathTracker::local_path_callback(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
	path_msg_ = *msg;
}

void PathTracker::timer_callback()
{
	if(!path_msg_.points.empty() && odometry_recieved_)
	{

		// initialization for state values
    double x_init[4];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    
    // initial value for control input
    double u0[1];
    u0[0] = 0.0;

    // prepare evaluation
    double min_time = 1e12;
    double elapsed_time;

    double xtraj[164];
    double utraj[40];

    // set velocity
    vel = new double(std::max(0.1, odometry_.twist.twist.linear.x));

		//RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "START POINT: x:%f, y:%f, v:%f, th:%f", x_init[0], x_init[1], x_init[2], x_init[3]);
		//RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "GOAL POINT: x:%f, y:%f, v:%f, th:%f", yref[0], yref[1], yref[2], yref[3]);

    // solve ocp in loop
		// initial condition
		ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_init);
  	ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_init);

    // Set Reference
    int path_length = path_msg_.points.size();
    int end_idx = std::min(N, path_length);

    double yref[4];
		yref[0] = path_msg_.points.at(end_idx-1).pose.position.x;
		yref[1] = path_msg_.points.at(end_idx-1).pose.position.y;
		yref[3] = tf2::getYaw(path_msg_.points.at(end_idx-1).pose.orientation);
    yref[4] = 0.0;

		for(int i = 0; i <= N; ++i)
		{
      ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
		}

    for(int i = 0; i < end_idx; ++i)
		{
      double y_intermediate[4];

      // setting intermediate traj
      y_intermediate[0] = path_msg_.points.at(i).pose.position.x;
      y_intermediate[1] = path_msg_.points.at(i).pose.position.y;
      y_intermediate[2] = tf2::getYaw(path_msg_.points.at(i).pose.orientation);
      y_intermediate[3] = 0.0;
    
      ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", y_intermediate);
		}
  
		for (int i = 0; i < N; i++)
    {
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
      nmpc_kinematic_controller_acados_update_params(acados_ocp_capsule, i, vel, NMPC_KINEMATIC_CONTROLLER_NP);
    }   

    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
    status = nmpc_kinematic_controller_acados_solve(acados_ocp_capsule);
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
			RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "nmpc_planner_acados_solve(): SUCCESS!");
    }
    else
    {
			RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "nmpc_planner_acados_solve() failed with status %d.\n", status);
    }

		autoware_planning_msgs::msg::Trajectory local_path = getLocalPathFromMPC(xtraj, utraj);

    // Set Longitudinal msg from local trajectory
		autoware_control_msgs::msg::Longitudinal longitudinal_msg = setVelocity(path_msg_);

    // Set Lateral msg from MPC
    autoware_control_msgs::msg::Lateral lateral_msg = setSteering(utraj);

		// Publish the trajectory
		trajectory_pub_->publish(local_path);

    // Publish control command
    autoware_control_msgs::msg::Control control_cmd;
    control_cmd.longitudinal = longitudinal_msg;
    control_cmd.lateral = lateral_msg;
    control_cmd.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    control_cmd_pub_->publish(control_cmd);

		// Publish visualization
		visualization_msgs::msg::MarkerArray visual_local_path;
    createLocalPathMarker(local_path, visual_local_path);
		viz_local_path_pub_->publish(visual_local_path);

	}
}

void PathTracker::setMPCProblem()
{
  acados_ocp_capsule = nmpc_kinematic_controller_acados_create_capsule();
  // there is an opportunity to change the number of shooting intervals in C without new code generation
  N = NMPC_KINEMATIC_CONTROLLER_N;
  // allocate the array and fill it accordingly
  status = nmpc_kinematic_controller_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);
  if (status)
  {
      printf("nmpc_kinematic_controller_acados_create() returned status %d. Exiting.\n", status);
      exit(1);
  }

	nlp_config = nmpc_kinematic_controller_acados_get_nlp_config(acados_ocp_capsule);
	nlp_dims = nmpc_kinematic_controller_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = nmpc_kinematic_controller_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = nmpc_kinematic_controller_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = nmpc_kinematic_controller_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = nmpc_kinematic_controller_acados_get_nlp_opts(acados_ocp_capsule);

}

autoware_control_msgs::msg::Longitudinal PathTracker::setVelocity(const autoware_planning_msgs::msg::Trajectory& path)
{
	autoware_control_msgs::msg::Longitudinal longitudinal_msg;

  longitudinal_msg.stamp = path.header.stamp;
  longitudinal_msg.velocity = path.points.at(1).longitudinal_velocity_mps;
  longitudinal_msg.acceleration = path.points.at(0).acceleration_mps2;
  longitudinal_msg.is_defined_acceleration = true;
  longitudinal_msg.is_defined_jerk = false;

	return longitudinal_msg;

}

autoware_control_msgs::msg::Lateral PathTracker::setSteering(double inputs[])
{
  autoware_control_msgs::msg::Lateral lateral_msg;
  lateral_msg.steering_tire_angle = inputs[0];
  lateral_msg.is_defined_steering_tire_rotation_rate = false;

  return lateral_msg;
}

autoware_planning_msgs::msg::Trajectory PathTracker::getLocalPathFromMPC(double states[], double inputs[])
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
		auto traj_point_quat = createQuaternionFromYaw(states[(i*NX)+2]);
		traj_point_pose.orientation.w = traj_point_quat.w;
		traj_point_pose.orientation.x = traj_point_quat.x;
		traj_point_pose.orientation.y = traj_point_quat.y;
		traj_point_pose.orientation.z = traj_point_quat.z;
		
		traj_point.pose = traj_point_pose;

    if(i < horizon_length)
    {
      traj_point.front_wheel_angle_rad = inputs[i];
      traj_point.rear_wheel_angle_rad = 0.0;
      RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "OPT TRAJ: u0:%f", inputs[i]);
    }

		RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "OPT TRAJ: x:%f, y:%f, v:%f", states[(i*NX)], states[(i*NX)+1], states[(i*NX)+2]);
		

		local_traj.points.push_back(traj_point);
	}

	return local_traj;
}

geometry_msgs::msg::Quaternion PathTracker::createQuaternionFromYaw(const double& yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void PathTracker::createLocalPathMarker(const autoware_planning_msgs::msg::Trajectory &lane_waypoints_array, 
																		    visualization_msgs::msg::MarkerArray& markerArray)
{
  std_msgs::msg::ColorRGBA total_color;
  total_color.r = 0.0;
  total_color.g = 0.2;
  total_color.b = 1.0;
  total_color.a = 1.0;
   
  visualization_msgs::msg::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "base_link";
  lane_waypoint_marker.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  lane_waypoint_marker.ns = "local_trajectory_marker";
  lane_waypoint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.5;
  lane_waypoint_marker.scale.y = 0.5;
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

}