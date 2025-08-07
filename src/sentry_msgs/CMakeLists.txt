cmake_minimum_required(VERSION 3.8)
project(sentry_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMsg.msg"
  "srv/CustomSrv.srv"
  "action/CustomAction.action"
  DEPENDENCIES geometry_msgs nav_msgs action_msgs
)

ament_package()