# Lanelet2 Mission Planner #

![](media/lanelet2_planner.gif)

## Objective ##
This repo contains a global mission planner module specific to the lanelet2 maps. It generates a path from a starting point to a goal point.

## Dependencies ##
**[Lanelet2 Library](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)** - Uses C++ API alonge with Autoware's utility functions based on lanelet2.

Autoware's utility functions are found [here](https://github.com/rohanNkhaire/SimplyAutonomous_utilities/tree/main/lanelet2_extension).

**This package is integrated with the [SimplyAutonomous](https://github.com/rohanNkhaire/SimplyAutonomous) software stack. Please follow the instructions in that repo to install the package**.

**The dependencies are installed with the installation of [SimplyAutonomous](https://github.com/rohanNkhaire/SimplyAutonomous) package**.

## Usage ##
```bash
# source the SimplyAutonomous workspace
source install/setup.bash

# Run the mission planner
ros2 run lanelet2_mission_planner lanelet2_mission_planner
```

## ROS ##

### Input topics ###

| Name                          | Type                  |        Description |
|:------------------------------|:----------------------|:------------------|
|/localization/kinematic_state  |nav_msgs::msg::Odometry| ego vehicle info |
|/goal_pose  |geometry_msgs::msg::PoseStamped| goal pose from RVIZ |
|/map/vector_map                |autoware_auto_mapping_msgs::msg::HADMapBin | lanelet2 map |



### Output topics ###

| Name                          | Type                  |        Description |
|:------------------------------|:----------------------|:------------------|
|/lanelet_mission_planner/path               |autoware_planning_msgs::msg::Trajectory| global planner traj |
|/visual__path              |visualization_msgs::msg::MarkerArray| global path viz |

## Note ##
This planner generates a path on the *centerline* of the lane. It takes the nearest point from the vehicle position as the start posiiton.


