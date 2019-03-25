cmake_minimum_required(VERSION 3.5)
project(nav2_actions)

find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(test_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

add_executable(test_client
  src/test_client.cpp
)

add_executable(test_server
  src/test_server.cpp
)

set(dependencies
  action_msgs
  nav2_msgs
  rclcpp_action
  rclcpp
  std_msgs
  test_msgs
)

ament_target_dependencies(test_client
  ${dependencies}
)

ament_target_dependencies(test_server
  ${dependencies}
)

install(TARGETS test_client test_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
