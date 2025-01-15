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
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(toggle_led_server src/toggle_LED_server.cpp)
ament_target_dependencies(toggle_led_server rclcpp std_srvs)
install(TARGETS
  toggle_led_server
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(toggle_led_client src/toggle_LED_client.cpp)
ament_target_dependencies(toggle_led_client rclcpp std_srvs)
install(TARGETS
  toggle_led_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()