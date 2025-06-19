cmake_minimum_required(VERSION 3.10)
project(gcs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(async_pac_gnn_interfaces REQUIRED)

set(target_dependencies
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  async_pac_gnn_interfaces
)

# Executables
add_executable(status_pac src/status_pac.cpp)
target_include_directories(status_pac PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(status_pac ${target_dependencies})
target_compile_features(status_pac PUBLIC c_std_99 cxx_std_17)

add_executable(mission src/mission_control.cpp)
target_include_directories(mission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mission ${target_dependencies})
target_compile_features(mission PUBLIC c_std_99 cxx_std_17)

add_executable(fake_robot src/fake_robot.cpp)
target_include_directories(fake_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fake_robot ${target_dependencies})
target_compile_features(fake_robot PUBLIC c_std_99 cxx_std_17)

# Composable nodes library
add_library(mission_component SHARED src/mission_components.cpp)
target_compile_definitions(mission_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(mission_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mission_component ${target_dependencies})
target_compile_features(mission_component PUBLIC c_std_99 cxx_std_17)

# Register the composable nodes
rclcpp_components_register_nodes(mission_component
  "gcs::MissionOriginGPS"
  "gcs::MissionControl"
)

# Install targets
install(TARGETS status_pac mission fake_robot
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS mission_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
