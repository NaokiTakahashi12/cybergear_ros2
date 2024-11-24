cmake_minimum_required(VERSION 3.16)
project(cybergear_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  cybergear_driver_core
  ros2_socketcan
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

add_library(cybergear_hardware_interface SHARED 
  src/cybergear_hardware_interface.cpp
)
ament_target_dependencies(cybergear_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(cybergear_hardware_interface PRIVATE cybergear_driver_core)
target_include_directories(cybergear_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(cybergear_hardware_interface PUBLIC
  cxx_std_20
)

# Export hardware plugins
pluginlib_export_plugin_description_file(
   hardware_interface cybergear_control.xml)

install(TARGETS cybergear_hardware_interface
  EXPORT export_cybergear_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  install(
    DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

  # find_package(ament_cmake_gmock REQUIRED)
  # ament_add_gmock(test_cybergear_interface test/test_load_cybergear_hardware_interface.cpp)
  # ament_target_dependencies(test_cybergear_interface hardware_interface)

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

ament_export_targets(export_cybergear_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(
  include
)
ament_package()
