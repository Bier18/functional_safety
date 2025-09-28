cmake_minimum_required(VERSION 3.8)
project(safety_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(safety_msgs REQUIRED)
find_package(std_msgs)
find_package(geometry_msgs)
find_package(pluginlib REQUIRED)

add_library(safety_manager SHARED
  src/HGMode.cpp
  src/PFLMode.cpp
  src/SRSMode.cpp
  src/SSMMode.cpp
)

target_include_directories(safety_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(safety_manager
  rclcpp
  std_msgs
  geometry_msgs
  safety_msgs
  pluginlib
)

install(TARGETS safety_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(safety_manager plugins.xml)

add_executable(state_machine src/StateMachine.cpp)

target_include_directories(state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(state_machine 
  rclcpp 
  safety_msgs 
  pluginlib 
  std_msgs 
  geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS state_machine
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_export_include_directories(
  include
)
ament_package()