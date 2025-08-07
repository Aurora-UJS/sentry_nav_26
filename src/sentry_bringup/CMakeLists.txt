cmake_minimum_required(VERSION 3.8)
project(sentry_bringup)

find_package(ament_cmake REQUIRED)
find_package(sentry_sim REQUIRED)
find_package(rviz2 REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(nav2_bringup REQUIRED)

install(DIRECTORY
  config/
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()