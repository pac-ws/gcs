cmake_minimum_required(VERSION 3.8)
project(gcs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

if (NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(target_dependencies
  rclcpp
  std_msgs
  geometry_msgs)

add_executable(status_pac src/status_pac.cpp)
target_include_directories(status_pac PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(status_pac ${target_dependencies})
target_compile_features(status_pac PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(mission_origin_gps src/mission_origin_gps.cpp)
target_include_directories(mission_origin_gps PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mission_origin_gps ${target_dependencies})
target_compile_features(mission_origin_gps PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS status_pac
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS mission_origin_gps
  DESTINATION lib/${PROJECT_NAME})

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
