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

## Note ##
This planner generates a path on the *centerline* of the lane. It takes the nearest point from the vehicle position as the start posiiton.


