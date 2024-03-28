cmake_minimum_required(VERSION 3.8)
project(nmpc_path_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# Acados
set(acados_include "/home/rohan/tools/acados/include")
set(acados_lib "/home/rohan/tools/acados/lib")
set(kinematic_model_build ${PROJECT_SOURCE_DIR}/scripts/kinematic/c_generated_code)

include_directories(
            ${kinematic_model_build}
            ${acados_include}
            ${acados_include}/blasfeo/include/
            ${acados_include}/hpipm/include/
            ${acados_include}/acados/
            ${acados_include}/qpOASES_e/
)

link_directories(${kinematic_model_build})

add_executable(nmpc_controller_kinematic src/kinematic/nmpc_controller.cpp
                            src/kinematic/nmpc_controller_node.cpp)                                
                                  
install(TARGETS nmpc_controller_kinematic
DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(nmpc_controller_kinematic rclcpp autoware_planning_msgs 
                tf2 autoware_control_msgs visualization_msgs tf2_geometry_msgs)               

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY scripts/kinematic/c_generated_code/
    DESTINATION include/${PROJECT_NAME}/${CUR_DIRECTORY_NAME}/
    FILES_MATCHING PATTERN "*.h")


install(DIRECTORY scripts/kinematic/c_generated_code/
    DESTINATION lib 
    FILES_MATCHING PATTERN "*.so")   

target_link_libraries(nmpc_controller_kinematic 
                    ${kinematic_model_build}/libacados_ocp_solver_nmpc_kinematic_controller.so
                    ${acados_lib}/libacados.so
) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
