cmake_minimum_required(VERSION 3.8)
project(mpc_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(mpc_plugin REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mpc++ CONFIG REQUIRED)

include_directories(include)
include_directories(${mpc++_INCLUDE_DIRS})

# Acados
set(acados_include "/home/rohan/tools/acados/include")
set(acados_lib "/home/rohan/tools/acados/lib")
set(full_model_build ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

include_directories(
            ${full_model_build}
            ${acados_include}
            ${acados_include}/blasfeo/include/
            ${acados_include}/hpipm/include/
            ${acados_include}/acados/
            ${acados_include}/qpOASES_e/
)

link_directories(${full_model_build})

add_executable(mpc_planner src/mpc_planner.cpp
                            src/mpc_planner_node.cpp)

add_executable(mpc_planner_acados src/mpc_planner_acados.cpp
                                  src/mpc_planner_acados_node.cpp)    
                                  
                                  

install(TARGETS mpc_planner
DESTINATION lib/${PROJECT_NAME})

install(TARGETS mpc_planner_acados
DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(mpc_planner rclcpp geometry_msgs autoware_planning_msgs 
                tf2_ros tf2 tf2_geometry_msgs lanelet2_extension mpc_plugin pluginlib)

ament_target_dependencies(mpc_planner_acados rclcpp geometry_msgs autoware_planning_msgs 
                tf2_ros tf2 tf2_geometry_msgs lanelet2_extension)                

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY scripts/c_generated_code/
    DESTINATION include/${PROJECT_NAME}/${CUR_DIRECTORY_NAME}/
    FILES_MATCHING PATTERN "*.h")


install(DIRECTORY scripts/c_generated_code/
    DESTINATION lib 
    FILES_MATCHING PATTERN "*.so")


target_link_libraries(${PROJECT_NAME} Eigen3::Eigen mpc++)

target_link_libraries(mpc_planner_acados 
                    ${full_model_build}/libacados_ocp_solver_nmpc_planner.so
                    ${acados_lib}/libacados.so
) 

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

ament_export_include_directories(include)

ament_package()
