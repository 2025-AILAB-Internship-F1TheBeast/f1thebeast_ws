cmake_minimum_required(VERSION 3.8)
project(global_path_optimizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(path_optimizer_node 
  src/path_optimizer_node.cpp
  src/MinCurvOptimizer.cpp
  src/MinCurvOptimizerQP.cpp
)

# Link dependencies
ament_target_dependencies(path_optimizer_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(path_optimizer_node
  Eigen3::Eigen
)

# Install targets
install(TARGETS
  path_optimizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  params/
  DESTINATION share/${PROJECT_NAME}/params
)

install(DIRECTORY
  inputs/
  DESTINATION share/${PROJECT_NAME}/inputs
)

ament_package()
