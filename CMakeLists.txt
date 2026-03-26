cmake_minimum_required(VERSION 3.8)
project(target_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# Find OpenCV
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(tracker_node src/target_tracker_node.cpp)

# Add the Control Executable [NEW]
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp geometry_msgs)

# Link everything together
ament_target_dependencies(tracker_node rclcpp sensor_msgs cv_bridge std_msgs geometry_msgs OpenCV)

# Install the executable
install(TARGETS
  tracker_node
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
