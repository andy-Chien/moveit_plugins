cmake_minimum_required(VERSION 3.8)
project(planning_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  mr_msgs
  std_msgs
  shape_msgs
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  geometric_shapes
  moveit_core
  moveit_ros_planning
  pluginlib
)

## System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/add_trajectory_obstacles.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

pluginlib_export_plugin_description_file(
  moveit_core "planning_adapter_plugin_description.xml")


ament_package()
