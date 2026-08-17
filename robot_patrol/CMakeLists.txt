cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Create executable
add_executable(patrol_exec src/patrol.cpp)
add_executable(test_service src/test_service.cpp)
add_executable(direction_service src/direction_service.cpp)
add_executable(patrol_with_service src/patrol_with_service.cpp)
add_executable(go_to_pose src/go_to_pose_action.cpp)

# Add dependencies
ament_target_dependencies(patrol_exec 
	rclcpp
	sensor_msgs)

ament_target_dependencies(patrol_with_service 
	rclcpp
	sensor_msgs
	custom_interfaces
)

ament_target_dependencies(go_to_pose 
	rclcpp
	sensor_msgs
	custom_interfaces
	action_msgs
	nav2_msgs
	rclcpp_action
	tf2
	tf2_geometry_msgs
)

# Add dependencies
ament_target_dependencies(direction_service 
	rclcpp
	sensor_msgs
	std_srvs
	custom_interfaces
)

# Add dependencies
ament_target_dependencies(test_service 
	rclcpp
	sensor_msgs
	std_srvs
	custom_interfaces
)

# Install executables
install(TARGETS 
	patrol_exec
	patrol_with_service
	direction_service
	test_service
  	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/)

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
