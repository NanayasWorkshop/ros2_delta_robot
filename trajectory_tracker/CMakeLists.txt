cmake_minimum_required(VERSION 3.8)
project(trajectory_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# Install Python scripts
install(PROGRAMS
  trajectory_tracker/trajectory_tracker_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME trajectory_tracker_node
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
