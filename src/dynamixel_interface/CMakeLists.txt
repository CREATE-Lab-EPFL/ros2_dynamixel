cmake_minimum_required(VERSION 3.8)
project(dynamixel_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include Dynamixel SDK
include_directories(/opt/ros/humble/include/dynamixel_sdk)
link_directories(/opt/ros/humble/lib)

# Add executable
add_executable(dynamixel_node src/dynamixel_node.cpp)
ament_target_dependencies(dynamixel_node rclcpp std_msgs)
target_link_libraries(dynamixel_node dynamixel_sdk)

# Install
install(TARGETS
  dynamixel_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()