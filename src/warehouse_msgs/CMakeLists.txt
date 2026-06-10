cmake_minimum_required(VERSION 3.8)
project(warehouse_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/RobotStatus.msg"
  "msg/TaskRequest.msg"
  "msg/FleetStatus.msg"
  "msg/TaskResult.msg"
  # Services
  "srv/AssignTask.srv"
  "srv/GetRobotStatus.srv"
  "srv/CancelTask.srv"
  # Actions
  "action/ExecuteTask.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
