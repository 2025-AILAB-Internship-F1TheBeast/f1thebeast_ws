cmake_minimum_required(VERSION 3.8)
project(control_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable - control.cpp 추가
add_executable(control_real_node src/control_real_node.cpp src/control_real.cpp)
add_executable(visualize_global_node src/visualize_global_node.cpp src/visualize_global.cpp)

ament_target_dependencies(control_real_node
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2 tf2_geometry_msgs custom_msgs
  tf2_ros
)

ament_target_dependencies(visualize_global_node 
  rclcpp geometry_msgs visualization_msgs
)

# Install Cpp executables
install(TARGETS
  control_real_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  visualize_global_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (추가)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files (추가 - 필요시)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()