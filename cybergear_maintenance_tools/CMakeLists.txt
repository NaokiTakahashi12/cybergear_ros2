cmake_minimum_required(VERSION 3.8)
project(cybergear_maintenance_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(cybergear_driver_core REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(search_cybergear_id_node_parameters
  src/search_cybergear_id_node_parameters.yaml
)
generate_parameter_library(change_cybergear_id_node_parameters
  src/change_cybergear_id_node_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME}_components SHARED
  src/search_cybergear_id_node.cpp
  src/change_cybergear_id_node.cpp
)
target_link_libraries(${PROJECT_NAME}_components
  search_cybergear_id_node_parameters
  cybergear_driver_core
  change_cybergear_id_node_parameters
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "cybergear_maintenance_tools::SearchCybergearIdNode"
  EXECUTABLE search_cybergear_id_node
)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "cybergear_maintenance_tools::ChangeCybergearIdNode"
  EXECUTABLE change_cybergear_id_node
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

ament_package()
