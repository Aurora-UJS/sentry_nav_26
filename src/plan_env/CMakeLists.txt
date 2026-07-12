cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
find_package(ament_cmake_auto REQUIRED)
# find dependencies
ament_auto_find_build_dependencies()
###########
## Build ##
###########

find_package(PCL REQUIRED COMPONENTS common filters search kdtree features)
find_package(OpenMP)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src 
)
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
  # NormalEstimationOMP 是头文件模板，在本包实例化，需要本包开 OpenMP 才真并行
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()


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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_occ_timeout
    test/test_occ_timeout.cpp
    src/map_core.cpp
  )
  target_include_directories(test_occ_timeout PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)