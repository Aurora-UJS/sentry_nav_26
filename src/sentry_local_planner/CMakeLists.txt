cmake_minimum_required(VERSION 3.8)
project(sentry_local_planner)

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
find_package(NLopt REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(sentry_local_planner_node
  src/sentry_local_planner_node.cpp
  src/trajectory_tracker.cpp
  src/minco_trajectory.cpp
)
target_include_directories(sentry_local_planner_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(sentry_local_planner_node
  rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs tf2_ros
  plan_env path_searching
)
target_link_libraries(sentry_local_planner_node ${NLOPT_LIBRARIES})

add_executable(chassis_cmd_node src/chassis_cmd_node.cpp)
target_include_directories(chassis_cmd_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(chassis_cmd_node
  rclcpp nav_msgs geometry_msgs sensor_msgs
)

install(TARGETS sentry_local_planner_node chassis_cmd_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_planner_fsm test/test_planner_fsm.cpp)
  target_include_directories(test_planner_fsm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_chassis_yaw_estimator test/test_chassis_yaw_estimator.cpp)
  target_include_directories(test_chassis_yaw_estimator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_safety_monitor
    test/test_safety_monitor.cpp
    src/minco_trajectory.cpp
  )
  target_include_directories(test_safety_monitor PRIVATE
    ${EIGEN3_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_safety_monitor plan_env)
  target_link_libraries(test_safety_monitor ${NLOPT_LIBRARIES})

  ament_add_gtest(test_trajectory_tracker
    test/test_trajectory_tracker.cpp
    src/trajectory_tracker.cpp
    src/minco_trajectory.cpp
  )
  target_include_directories(test_trajectory_tracker PRIVATE
    ${EIGEN3_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_trajectory_tracker plan_env geometry_msgs)
  target_link_libraries(test_trajectory_tracker ${NLOPT_LIBRARIES})
endif()

ament_package()
