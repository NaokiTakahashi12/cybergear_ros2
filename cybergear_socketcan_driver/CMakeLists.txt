cmake_minimum_required(VERSION 3.8)
project(cybergear_socketcan_driver
    VERSION 0.0.0
    LANGUAGES
      CXX
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(cybergear_driver_core REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(cybergear_socketcan_driver_node_params
  src/cybergear_socketcan_driver_node_params.yaml
)

ament_auto_add_library(${PROJECT_NAME}_components SHARED
  src/cybergear_socketcan_driver_node.cpp
)
target_compile_features(${PROJECT_NAME}_components
  PUBLIC
    cxx_std_20
)
target_link_libraries(${PROJECT_NAME}_components
  cybergear_socketcan_driver_node_params
  cybergear_driver_core::cybergear_driver_core
)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "cybergear_socketcan_driver::CybergearSocketCanDriverNode"
  EXECUTABLE cybergear_socketcan_driver_node
)

install(
  TARGETS
    ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
