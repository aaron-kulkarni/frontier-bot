cmake_minimum_required(VERSION 3.8)
project(turtlebot3_autonav)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Include directories
include_directories(
  include
)

# Explorer node
add_executable(explorer_node
  src/explorer_node.cpp
)
ament_target_dependencies(explorer_node
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_msgs
  rclcpp_action
)
target_link_libraries(explorer_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

install(TARGETS explorer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
