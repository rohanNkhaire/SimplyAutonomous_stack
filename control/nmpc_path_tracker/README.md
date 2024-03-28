# Non-Linear Model Predictive Control Path Tracker #

**This repo is work in progress**

## Objective ##
This repo solves the path tracking problem via non-linear model predictive control scheme

## Dependencies ##
- [Acados](https://github.com/acados/acados) - Please follow the instructions from their GitHub page.

## ROS ##

### Input topics ###

| Name                          | Type                  |        Description |
|:------------------------------|:----------------------|:------------------|
|/localization/kinematic_state  |nav_msgs::msg::Odometry| ego vehicle info |
|/mpc_planner/traj  |autoware_planning_msgs::msg::Trajectory| local trajectory |
|/map/vector_map                |autoware_auto_mapping_msgs::msg::HADMapBin | lanelet2 map |


## Note ##
**This repo works on top of the local trajectory planner [mpc_planner](https://github.com/rohanNkhaire/SimplyAutonomous_stack/tree/main/planning/mpc_planner)**.