cmake_minimum_required(VERSION 3.8)
project(sentry_control)

# Enable compile_commands.json generation for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(sentry_nav_decision_interface REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

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

add_executable(sentry_communication src/sentry_communication.cpp)
target_include_directories(sentry_communication PRIVATE include)
ament_target_dependencies(sentry_communication
 rclcpp 
 geometry_msgs 
 serial 
 sensor_msgs
 tf2
 auto_aim_interfaces
 sentry_nav_decision_interface
 )
install(TARGETS sentry_communication
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(sentry_nav_goal src/sentry_nav_goal.cpp)
target_include_directories(sentry_nav_goal PRIVATE include)
ament_target_dependencies(sentry_nav_goal
 rclcpp 
 geometry_msgs  
 sentry_nav_decision_interface
 nav2_msgs
 rclcpp_action
 )
install(TARGETS sentry_nav_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
