#include "mpc_planner/mpc_planner.hpp"

MPCPlanner::MPCPlanner() : Node("mpc_planner")
{
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", rclcpp::QoS{10},
																				std::bind(&MPCPlanner::odom_callback, this, std::placeholders::_1));

	global_path_sub_ = create_subscription<autoware_planning_msgs::msg::Path>("/lanelet_mission_planner/path", rclcpp::QoS{1},
																									std::bind(&MPCPlanner::global_path_callback, this, std::placeholders::_1));

	const auto durable_qos = rclcpp::QoS(1).transient_local();
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>("/map/vector_map", durable_qos,
																								std::bind(&MPCPlanner::map_callback, this, std::placeholders::_1));

  trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("/mpc_planner/traj", rclcpp::QoS{10});

	// Timer
	auto timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&MPCPlanner::timer_callback, this));

	// Init NMPC
	setMPCProblem();
																																												
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

void MPCPlanner::map_callback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr& msg)
{
  map_ptr_ = msg;

  // Creating lanelet map
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
}

void MPCPlanner::global_path_callback(const autoware_planning_msgs::msg::Path::ConstSharedPtr& msg)
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

		// Set the goal point
		geometry_msgs::msg::Pose goal_pose = setGoal(path_msg_, odometry_.twist.twist.linear.x, curr_index);

		// Get Goal Yaw
    tf2::Quaternion goal_quat_;
    tf2::Matrix3x3 m;
    tf2::fromMsg(goal_pose.orientation, goal_quat_);
    m.setRotation(goal_quat_);
    double goal_roll, goal_pitch, goal_yaw;
    m.getRPY(goal_roll, goal_pitch, goal_yaw);

		// Set the reference for MPC
		yref(0) = goal_pose.position.x;
		yref(1) = goal_pose.position.y;
		yref(3) = goal_yaw;
		yref(4) = 5.0;

		// Get Vehicle Yaw
    tf2::Quaternion veh_quat_;
    tf2::Matrix3x3 m;
    tf2::fromMsg(odometry_.pose.pose.orientation, veh_quat_);
    m.setRotation(veh_quat_);
    double veh_roll, veh_pitch, veh_yaw;
    m.getRPY(veh_roll, veh_pitch, veh_yaw);

		// Initialize MPC
		mpc::cvec<num_states> modelX, modeldX;

  	modelX.resize(num_states);
  	modeldX.resize(num_states);
  	modelX(0) = odometry_.pose.pose.position.x;
  	modelX(1) = odometry_.pose.pose.position.y;
		modelX(2) = veh_yaw;
		modelX(3) = odometry_.twist.twist.linear.x;

		auto r = controller.getLastResult();

		r = controller.step(modelX, r.cmd);

		std::cout << controller.getExecutionStats();
	}
}

void MPCPlanner::setMPCProblem()
{


	controller.setLoggerLevel(mpc::Logger::log_level::NORMAL);
  controller.setContinuosTimeModel(ts);		

	auto stateEq = [&](
                       mpc::cvec<num_states> &dx,
                       const mpc::cvec<num_states> &x,
                       const mpc::cvec<num_inputs> &u)
    {
        dx(0) = x(3)*cos(x(2));
        dx(1) = x(3)*sin(x(2));
				dx(2) = u(0);
				dx(3) = u(1);
    };

		controller.setStateSpaceFunction([&](
                                        mpc::cvec<num_states> &dx,
                                        const mpc::cvec<num_states> &x,
                                        const mpc::cvec<num_inputs> &u,
                                        const unsigned int &)
                                    { stateEq(dx, x, u); });

    controller.setObjectiveFunction([&](
                                       const mpc::mat<pred_hor + 1, num_states> &x,
                                       const mpc::mat<pred_hor + 1, num_output> &,
                                       const mpc::mat<pred_hor + 1, num_inputs> &u,
                                       double)
                                   { 
																			double cost = 0;
        							for (int i = 0; i < (pred_hor + 1); ++i)
        							{
        							    cost += (x.row(i).segment(0, 2).transpose() - yref.segment(0, 2)).squaredNorm();
        							    cost += u.row(i).squaredNorm();
													cost += std::norm(x(i, 3) - yref(3));
													cost += std::norm(x(i, 2) - yref(2));
													cost += i > 0 ? std::norm(u(i) - u(i-1)) : 0.0;
        							}

        							return cost;
									
									
									return x.array().square().sum() + u.array().square().sum(); });

																 
}

geometry_msgs::msg::Pose MPCPlanner::setGoal(autoware_planning_msgs::msg::Path& path, double& curr_vel, int& curr_idx)
{
	// Set goal such that we plan for 4s minimum
	// We take goal from the global planner
	// scale goal with vehicle speed
	int max_length = path.points.size() - 1;
	int min_goal_idx = std::min(curr_idx + MIN_GOAL_IDX, max_length);
	int goal_idx = std::min(curr_idx + int(std::round(curr_vel*LOOK_AHEAD_TIME)), min_goal_idx);
	geometry_msgs::msg::Pose goal_pose = path.points.at(goal_idx).pose;

	return goal_pose;
}

int getCurrentIndex(std::vector<autoware_planning_msgs::msg::PathPoint>& path_point, nav_msgs::msg::Odometry& veh_odom)
{
	int curr_index = findNearestIndex(path_point, veh_odom.pose.pose.position);
	return curr_index;
}

size_t findNearestIndex(std::vector<autoware_planning_msgs::msg::PathPoint>& points, const geometry_msgs::msg::Point& point)
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

double calcSquaredDistance2d(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2)
{
  const auto p1 = point1;
  const auto p2 = point2;
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;
  return dx * dx + dy * dy;
}