cmake_minimum_required(VERSION 3.8)
project(move_group_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_capabilities REQUIRED)
find_package(moveit_task_constructor_demo REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_task_constructor_visualization REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare a C++ executable
# step 1
add_executable(moveit_arm src/moveit_arm.cpp)
# step 2
  ament_target_dependencies(moveit_arm
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_capabilities
  moveit_task_constructor_demo
  moveit_task_constructor_msgs
  moveit_task_constructor_visualization
)
# step 3
install(TARGETS moveit_arm
  DESTINATION lib/${PROJECT_NAME}
)

# joint state relay
# step 1
add_executable(joint_states_relay src/joint_states_relay.cpp)
# step 2
  ament_target_dependencies(joint_states_relay
  rclcpp
  sensor_msgs
)
# step 3
install(TARGETS joint_states_relay
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
