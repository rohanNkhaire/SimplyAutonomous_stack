#include "mpc_planner/mpc_planner.hpp"

MPCPlanner::MPCPlanner() : Node("mpc_planner")
{
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", rclcpp::QoS{10},
																				std::bind(&MPCPlanner::odom_callback, this, std::placeholders::_1));
	const auto durable_qos = rclcpp::QoS(1).transient_local();
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>("/map/vector_map", durable_qos,
																								std::bind(&MPCPLanner::map_callback, this, std::placeholders::_1));

  trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("/mpc_planner/traj", rclcpp::QoS{10});

	// Timer
	auto timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&MPCPlanner::timer_callback, this))
																																												
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

void MPCPlanner::timer_callback()
{
	// Initialize MPC
	mpc::cvec<num_states> modelX, modeldX;

  modelX.resize(num_states);
  modeldX.resize(num_states);
  modelX(0) = odometry_.pose.pose.position.x;
  modelX(1) = odometry_.pose.pose.position.y;
	modelX(2) = odometry_.twist.twist.linear.x;
	modelX(3) = odometry_.twist.twist.angular.z;

	auto r = controller.getLastResult();

	r = controller.step(modelX, r.cmd);

	std::cout << controller.getExecutionStats();
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
        dx(0) = x(4)*cos(x(3));
        dx(1) = x(4)*sin(x(3));
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
                                   { return x.array().square().sum() + u.array().square().sum(); });

																 
}