cmake_minimum_required(VERSION 3.8)
project(sentry_global_planner)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(sentry_global_planner_node src/global_planner_node.cpp)
target_include_directories(sentry_global_planner_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(sentry_global_planner_node
  rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs plan_env
)
target_link_libraries(sentry_global_planner_node ${OpenCV_LIBRARIES})

install(TARGETS sentry_global_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
