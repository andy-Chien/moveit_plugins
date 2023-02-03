cmake_minimum_required(VERSION 3.8)
project(scene_buffer)

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
  example_interfaces
)

## System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/scene_buffer.cpp
)
target_include_directories(${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(${PROJECT_NAME}_node
  src/scene_buffer_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_features(scene_buffer_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_node
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
