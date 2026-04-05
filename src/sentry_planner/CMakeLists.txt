cmake_minimum_required(VERSION 3.8)
project(sentry_planner)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(NLopt REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(sentry_planner_node src/sentry_planner_node.cpp)
target_include_directories(sentry_planner_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(sentry_planner_node
  rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs tf2_ros
  plan_env path_searching bspline bspline_opt
)
target_link_libraries(sentry_planner_node ${NLOPT_LIBRARIES})

install(TARGETS sentry_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
