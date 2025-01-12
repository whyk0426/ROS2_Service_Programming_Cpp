cmake_minimum_required(VERSION 3.5)
project(ros2_cpp_tutorial)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(double_talker src/p1_publisher.cpp)
ament_target_dependencies(double_talker rclcpp std_msgs)
install(TARGETS
  double_talker
  DESTINATION lib/${PROJECT_NAME})

add_executable(double_listener src/p1_subscriber.cpp)
ament_target_dependencies(double_listener rclcpp std_msgs)
install(TARGETS
  double_listener
  DESTINATION lib/${PROJECT_NAME})

add_executable(fast_agent src/p2_fast_agent.cpp)
ament_target_dependencies(fast_agent rclcpp std_msgs)
install(TARGETS
  fast_agent
  DESTINATION lib/${PROJECT_NAME})

add_executable(slow_agent src/p2_slow_agent.cpp)
ament_target_dependencies(slow_agent rclcpp std_msgs)
install(TARGETS
  slow_agent
  DESTINATION lib/${PROJECT_NAME})

ament_package()