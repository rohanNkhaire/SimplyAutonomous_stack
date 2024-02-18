cmake_minimum_required(VERSION 3.8)
project(lanelet2_mission_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lanelet2_extension REQUIRED)

include_directories(include)
add_executable(lanelet2_mission_planner src/lanelet2_mission_planner.cpp
                                        src/lanelet2_mission_planner_node.cpp
                                        src/lanelet2_mission_planner_utils
                                        )

ament_target_dependencies(lanelet2_mission_planner rclcpp geometry_msgs autoware_planning_msgs 
                tf2_ros tf2 tf2_geometry_msgs lanelet2_extension)

install(TARGETS lanelet2_mission_planner
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
