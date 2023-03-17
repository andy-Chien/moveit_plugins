cmake_minimum_required(VERSION 3.22)
project(moveit_capability)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-overloaded-virtual)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(MOVEIT_LIB_NAME moveit_move_group_capabilities)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  moveit_ros_occupancy_map_monitor
  moveit_ros_planning
  pluginlib
)

## System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED system filesystem date_time program_options thread)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/move_action_capability.cpp
  src/async_execute_trajectory_action_capability.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    ${MOVEIT_LIB_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

pluginlib_export_plugin_description_file(
  moveit_ros_move_group "moveit_capabilities_plugin_description.xml")

ament_package()