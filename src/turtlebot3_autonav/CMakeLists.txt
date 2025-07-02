cmake_minimum_required(VERSION 3.8)
project(turtlebot3_autonav)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

# Add the main explorer node
add_executable(explorer_node src/explorer_node.cpp)
target_link_libraries(explorer_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  nav2_msgs::nav2_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  tf2::tf2
  tf2_ros::tf2_ros
)

install(TARGETS explorer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (if any)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install maps (if any)
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install Python scripts (if any)
# (initial_pose_pub.py is installed via setup.py as a console script)

ament_package()
