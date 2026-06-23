cmake_minimum_required(VERSION 3.8)
project(sentry_trav_rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 依赖 (Jazzy / Qt5 RViz 插件标准组合) ---
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# RViz 插件类带 Q_OBJECT, 必须经过 Qt MOC 处理
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 头文件必须列入目标源, 这样 AUTOMOC 才会对带 Q_OBJECT 的头跑 moc
set(${PROJECT_NAME}_headers
  include/sentry_trav_rviz_plugin/annotation_bridge.hpp
  include/sentry_trav_rviz_plugin/traversability_panel.hpp
  include/sentry_trav_rviz_plugin/traversability_tool.hpp
)
set(${PROJECT_NAME}_sources
  src/annotation_bridge.cpp
  src/traversability_panel.cpp
  src/traversability_tool.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_sources}
  ${${PROJECT_NAME}_headers}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Qt5::Widgets 与 yaml-cpp 是非 ament 包, 显式链接
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  rviz_rendering
  pluginlib
  geometry_msgs
  visualization_msgs
  rclcpp
)

# 向 rviz_common 注册插件描述 (Panel + Tool)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rviz_common
  rviz_rendering
  pluginlib
  geometry_msgs
  visualization_msgs
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
