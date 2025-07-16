cmake_minimum_required(VERSION 3.8)
project(system_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemCommand.msg"
  "msg/VehicleStatus.msg" 
  "msg/MissionStatus.msg"
  "srv/GetStatus.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# Create include directories
include_directories(include)

# System Manager Node
add_executable(system_manager_node src/system_manager_node.cpp src/policy_base.cpp)
ament_target_dependencies(system_manager_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(system_manager_node "${cpp_typesupport_target}")

# Command Input Arbiter Node
add_executable(command_input_arbiter_node src/command_input_arbiter_node.cpp)
ament_target_dependencies(command_input_arbiter_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(command_input_arbiter_node "${cpp_typesupport_target}")

# Vehicle Adapter Manager Node
add_executable(vehicle_adapter_manager_node src/vehicle_adapter_manager_node.cpp src/vehicle_type_base.cpp)
ament_target_dependencies(vehicle_adapter_manager_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vehicle_adapter_manager_node "${cpp_typesupport_target}")

# Telemetry Collector Node
add_executable(telemetry_collector_node src/telemetry_collector_node.cpp)
ament_target_dependencies(telemetry_collector_node rclcpp std_msgs sensor_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(telemetry_collector_node "${cpp_typesupport_target}")

# Mission Service Node
add_executable(mission_service_node src/mission_service_node.cpp src/mission_type_base.cpp)
ament_target_dependencies(mission_service_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mission_service_node "${cpp_typesupport_target}")

# Mission Adapter Manager Node
add_executable(mission_adapter_manager_node src/mission_adapter_manager_node.cpp src/payload_type_base.cpp)
ament_target_dependencies(mission_adapter_manager_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mission_adapter_manager_node "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  system_manager_node
  command_input_arbiter_node
  vehicle_adapter_manager_node
  telemetry_collector_node
  mission_service_node
  mission_adapter_manager_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Unit tests for core classes
  ament_add_gtest(test_core_classes test/test_core_classes.cpp src/policy_base.cpp src/vehicle_type_base.cpp src/mission_type_base.cpp src/payload_type_base.cpp)
  target_include_directories(test_core_classes PUBLIC include)
  ament_target_dependencies(test_core_classes rclcpp)
  
  # Regression tests
  ament_add_gtest(test_regression test/test_regression.cpp src/policy_base.cpp src/vehicle_type_base.cpp src/mission_type_base.cpp src/payload_type_base.cpp)
  target_include_directories(test_regression PUBLIC include)
  ament_target_dependencies(test_regression rclcpp)
  
  # ROS 2 Integration tests (requires running system)
  ament_add_gtest(test_ros2_integration test/test_ros2_integration.cpp)
  target_include_directories(test_ros2_integration PUBLIC include)
  ament_target_dependencies(test_ros2_integration rclcpp std_msgs)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_ros2_integration "${cpp_typesupport_target}")
  
  # Lint tests
  ament_lint_auto_find_test_dependencies()
endif()

# Install test files
install(DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test/)

ament_package() 