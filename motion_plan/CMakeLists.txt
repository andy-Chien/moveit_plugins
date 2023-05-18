cmake_minimum_required(VERSION 3.22)
project(motion_plan)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-overloaded-virtual)
endif()

# add_compile_options(-g)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_planning_interface
  rclcpp
  pluginlib
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  std_msgs
  std_srvs
  Eigen3
  ompl
)

set(THIS_PACKAGE_LIBRARIES
  ${PROJECT_NAME}_objectives
  ${PROJECT_NAME}_planner
  ${PROJECT_NAME}_ompl_tools
  moveit_ompl_interface
  moveit_ompl_planner_plugin
)

## System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  SYSTEM
  ompl/tools/include
  adapt_prm/include
  ompl_interface/include
  objectives/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_subdirectory(ompl)
add_subdirectory(adapt_prm)
add_subdirectory(objectives)
add_subdirectory(ompl_interface)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)


# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h*"
# )
install(TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core ompl)

pluginlib_export_plugin_description_file(moveit_core ompl_interface_plugin_description.xml)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
# endif()

ament_package()