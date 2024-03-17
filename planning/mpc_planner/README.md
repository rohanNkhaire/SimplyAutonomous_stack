# Non-Linear Model Predictive Control based Motion Planning #

## Objective ##
This repo contains a local planner based on NMPC scheme. It works on the lanelet2 mission planning module.

## Dependencies ##
**[Acados](https://github.com/acados/acados) - Please follow the installation of acados from their github page**.

**[libmcp++](https://github.com/nicolapiccinelli/libmpc) - Please follow the installation of libmpc++ from their github page**

```bash
# Install this library (taken care of if SimplyAutonomus repo is installed)
sudo apt install libnlopt0
```

## Acados Usage ##
```bash
# Generate the c code by running the python file
cd scripts
python3 generate_c_code.py

# Build the planner
# Update the paths to acados in the CmakeLists file
colcon build --packages-select mpc_planner

# Run the node with acados
source install/setup.bash
ros2 run mpc_planner mpc_planner_acados
```

## libmpc++ Usage ##
```bash
# Make sure the mpc_plugin is built first
colcon build --packages-select mpc_planner

# Run the node with libmpc++
ros2 run mpc_planner mpc_planner
```
## ROS ##

### Input topics ###

| Name                          | Type                  |        Description |
|:------------------------------|:----------------------|:------------------|
|/localization/kinematic_state  |nav_msgs::msg::Odometry| ego vehicle info |
|/lanelet_mission_planner/path  |autoware_planning_msgs::msg::Path| global planner path |
|/map/vector_map                |autoware_auto_mapping_msgs::msg::HADMapBin | lanelet2 map |



### Output topics ###

| Name                          | Type                  |        Description |
|:------------------------------|:----------------------|:------------------|
|/mpc_planner/traj               |autoware_planning_msgs::msg::Trajectory| local planner traj |
|/visual_local_path              |visualization_msgs::msg::MarkerArray| local path and goal viz |


## Note ##
**This repo works on top of the mission planner [lanelet2_mission_planner](https://github.com/rohanNkhaire/SimplyAutonomous_stack/tree/main/planning/lanelet2_mission_planner)**.

### Acados version ###
- **To make changes to the NMPC formulation, change the *generate_c_code.py* file**
- The mpc planner plans 4.0 sec ahead.
- The acceleration limit is 1ms-2

### libmpc++ version ###
I am still trying to get this to run properly.

## Reference ##
- [Path Planning for Autonomous Vehicles using Model Predictive Control](https://users.soe.ucsc.edu/~habhakta/MPC_pathPlanning.pdf) - Chang Liu, Seungho Lee, Scott Varnhagen, H. Eric Tseng