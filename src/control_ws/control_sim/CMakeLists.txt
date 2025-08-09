cmake_minimum_required(VERSION 3.8)
project(control_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE 
  ament_cmake
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  Eigen3)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable - control.cpp 추가
add_executable(control_sim_node src/control_sim_node.cpp src/control_sim.cpp)
add_executable(visualize_global_node src/visualize_global_node.cpp src/visualize_global.cpp)

ament_target_dependencies(control_sim_node
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2 tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(visualize_global_node 
  rclcpp geometry_msgs visualization_msgs
)

# Install Cpp executables
install(TARGETS
  control_sim_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  visualize_global_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()