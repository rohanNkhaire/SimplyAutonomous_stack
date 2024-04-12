cmake_minimum_required(VERSION 3.22)
project(pointcloud_preprocessor_gpu CXX CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(include
                    ${CUDAToolkit_INCLUDE_DIRS})  


add_executable(cropbox_filter src/filtering/filtering.cpp
                              src/filtering/filtering.cu)               

target_include_directories(cropbox_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDAToolkit_INCLUDE_DIRS})

target_compile_features(cropbox_filter PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(cropbox_filter rclcpp sensor_msgs tf2_geometry_msgs pcl_ros PCL CUDAToolkit)

install(TARGETS cropbox_filter
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(cropbox_filter ${CUDAToolkit_LIBRARY_DIR}/libcudart.so)

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
